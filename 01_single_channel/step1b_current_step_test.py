"""
1단계-b 실험: 정전류(CC) 응답 테스트 (한 세션 = 한 전류값)
- Step 1(정전압)과 반대로, 이번에는 "전류"를 고정하고 저항 변화에 따라 전압이 어떻게
  움직이는지 + 온도가 어떻게 반응하는지를 확인
- Chroma 62000P를 CC 모드로 강제 진입시키기 위해: SOUR:VOLT는 안전 상한(기본 24V)으로
  넉넉히 걸어두고, SOUR:CURR을 목표 전류로 설정함 → 발열체 저항이 있는 한 전류가
  먼저 그 설정값에 도달해서 CC 모드로 동작하게 됨 (FETC:STAT?로 실제 CC 진입 확인)
- MAX31856 + K타입 열전대로 온도를 동시에 로깅
- 세션 종료 시 CSV 저장 + 그래프(PNG, 온도 + 전압 두 축) 자동 생성 + 정상상태 평균 온도 계산

사용 방법:
    python3 step1b_current_step_test.py 1.0
    (인자로 목표 전류값(A) 하나를 넘김. 생략하면 아래 TARGET_CURRENT 기본값 사용)

한 세션 = 한 전류값입니다. 다음 전류를 테스트하기 전에는 발열체가 충분히
식은 걸 직접 눈으로 확인한 뒤 스크립트를 다시 실행하세요 (자동 냉각 대기 없음).

주의:
- 목표 전류가 너무 크면 저항이 낮은(차가운) 초반에 순간적으로 큰 전력이 걸릴 수 있음
- FETC:STAT?의 세 번째 값이 CC가 아니라 CV로 계속 나오면, 전압 상한(24V) 안에서는
  그 전류에 도달하지 못한다는 뜻 → 목표 전류를 낮추거나 전압 상한 재검토 필요

사용 전 반드시 확인:
1) Chroma 62000P 본체 Baud Rate가 38400으로 설정되어 있는지
2) MAX31856 배선 및 라이브러리 설치
     pip install adafruit-circuitpython-max31856 --break-system-packages
     pip install matplotlib --break-system-packages
3) PORT 경로 (ls /dev/ttyUSB*)
4) 전압/전류 상한 (24V / 5A) — 코드 내 SAFETY LIMIT 밖의 값은 절대 넣지 말 것
"""

import sys
import time
import csv
from datetime import datetime
from typing import Optional

import serial
from serial import SerialException

import board
import digitalio
import adafruit_max31856

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager

# --- 한글 폰트 설정 (없으면 영문 라벨로 자동 대체) ---
_KOREAN_FONT_CANDIDATES = [
    "NanumGothic", "NanumBarunGothic", "Noto Sans CJK KR", "Noto Sans KR", "UnDotum",
]
_installed_fonts = {f.name for f in font_manager.fontManager.ttflist}
KOREAN_FONT_AVAILABLE = False
for _name in _KOREAN_FONT_CANDIDATES:
    if _name in _installed_fonts:
        matplotlib.rcParams["font.family"] = _name
        KOREAN_FONT_AVAILABLE = True
        break
matplotlib.rcParams["axes.unicode_minus"] = False

if not KOREAN_FONT_AVAILABLE:
    print(
        "[안내] 한글 폰트를 찾지 못해 그래프 라벨을 영문으로 표시합니다.\n"
        "       sudo apt-get install -y fonts-nanum && rm -rf ~/.cache/matplotlib"
    )

# ============================================================
# 설정값
# ============================================================

TARGET_CURRENT = 1.0                # 이번 세션에서 고정할 전류 (A). 커맨드라인 인자로 덮어쓸 수 있음

PORT = "/dev/ttyUSB0"
BAUD_RATE = 38400

SAFETY_MAX_VOLTAGE = 24.0
SAFETY_MAX_CURRENT = 5.0

# --- CC 모드를 강제하기 위한 전압 상한 (이 전압 안에서 목표 전류에 도달하지 못하면 CV로 남음) ---
VOLTAGE_CEILING = 24.0

MAX_HOLD_SECONDS = 10 * 60
SAMPLE_INTERVAL_SECONDS = 2.0

TEMP_SAFETY_CUTOFF_C = 85.0

STEADY_STATE_WINDOW_SECONDS = 90.0
STEADY_STATE_EPSILON_C = 0.3

CS_PIN = board.D5
THERMOCOUPLE_TYPE = adafruit_max31856.ThermocoupleType.K


# ============================================================
# Chroma 62000P SCPI 통신 함수
# ============================================================

def write_command(device: serial.Serial, command: str) -> None:
    message = f"{command}\n".encode("ascii")
    device.write(message)
    device.flush()


def query(device: serial.Serial, command: str) -> str:
    write_command(device, command)
    response = device.readline()
    if not response:
        raise TimeoutError(f"장비 응답 시간 초과: {command}")
    return response.decode("ascii", errors="replace").strip()


def check_error(device: serial.Serial) -> None:
    err = query(device, "SYST:ERR?")
    if not err.startswith("0"):
        raise RuntimeError(f"장비 오류 발생: {err}")


def safe_shutdown(device: serial.Serial) -> None:
    try:
        write_command(device, "CONF:OUTP OFF")
    except Exception:
        pass


# ============================================================
# 결과 처리
# ============================================================

def compute_steady_state_average(times, temps, window_s: float):
    if not times:
        return None, None
    end_time = times[-1]
    window_vals = [t for tm, t in zip(times, temps) if end_time - tm <= window_s]
    if not window_vals:
        window_vals = temps[-1:]
    avg = sum(window_vals) / len(window_vals)
    variance = sum((v - avg) ** 2 for v in window_vals) / len(window_vals)
    std = variance ** 0.5
    return avg, std


_STOP_REASON_EN = {
    "정체(steady-state) 판정": "steady-state reached",
    "시간 초과": "timeout",
}


def save_plot(times, temps, volts, target_current, avg_temp, std_temp, stop_reason, out_path):
    if KOREAN_FONT_AVAILABLE:
        xlabel = "경과 시간 (s)"
        temp_label, volt_label = "온도 실측", "전압 실측 (CC 추종)"
        avg_label = f"정상상태 평균 {avg_temp:.2f}±{std_temp:.2f}C" if avg_temp is not None else ""
        title = f"{target_current}A 정전류 응답 — 종료 사유: {stop_reason}"
        ylabel_t, ylabel_v = "온도 (C)", "전압 (V)"
    else:
        xlabel = "Elapsed time (s)"
        temp_label, volt_label = "Measured temperature", "Measured voltage (CC tracking)"
        avg_label = f"Steady-state avg {avg_temp:.2f}±{std_temp:.2f}C" if avg_temp is not None else ""
        title = f"{target_current}A constant-current response — stop reason: {_STOP_REASON_EN.get(stop_reason, stop_reason)}"
        ylabel_t, ylabel_v = "Temperature (C)", "Voltage (V)"

    fig, ax1 = plt.subplots(figsize=(9, 5))
    ax1.plot(times, temps, color="#2b6cb0", linewidth=1.2, label=temp_label)
    if avg_temp is not None:
        ax1.axhline(avg_temp, color="#c53030", linestyle="--", linewidth=1, label=avg_label)
    ax1.set_xlabel(xlabel)
    ax1.set_ylabel(ylabel_t, color="#2b6cb0")
    ax1.tick_params(axis="y", labelcolor="#2b6cb0")
    ax1.grid(True, alpha=0.3)

    ax2 = ax1.twinx()
    ax2.plot(times, volts, color="#38a169", linewidth=1.0, alpha=0.7, label=volt_label)
    ax2.set_ylabel(ylabel_v, color="#38a169")
    ax2.tick_params(axis="y", labelcolor="#38a169")

    lines1, labels1 = ax1.get_legend_handles_labels()
    lines2, labels2 = ax2.get_legend_handles_labels()
    ax1.legend(lines1 + lines2, labels1 + labels2, loc="lower right")

    ax1.set_title(title)
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)


# ============================================================
# 메인 실험 루틴
# ============================================================

def main() -> None:
    target_current = TARGET_CURRENT
    if len(sys.argv) > 1:
        target_current = float(sys.argv[1])

    if target_current > SAFETY_MAX_CURRENT:
        raise ValueError(f"목표 전류({target_current}A)가 안전 상한(5A)을 초과합니다.")
    if VOLTAGE_CEILING > SAFETY_MAX_VOLTAGE:
        raise ValueError("전압 상한 설정이 안전 상한(24V)을 초과합니다.")

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_path = f"step1b_{target_current:g}A_{timestamp}.csv"
    plot_path = f"step1b_{target_current:g}A_{timestamp}.png"

    spi = board.SPI()
    cs = digitalio.DigitalInOut(CS_PIN)
    thermocouple = adafruit_max31856.MAX31856(spi, cs, thermocouple_type=THERMOCOUPLE_TYPE)

    power_supply: Optional[serial.Serial] = None
    times, temps, volts = [], [], []
    stop_reason = "시간 초과"
    warned_cv = False

    try:
        power_supply = serial.Serial(
            port=PORT, baudrate=BAUD_RATE,
            bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,
            timeout=2.0, write_timeout=2.0,
            xonxoff=False, rtscts=False, dsrdtr=False,
        )
        power_supply.reset_input_buffer()
        power_supply.reset_output_buffer()

        write_command(power_supply, "*CLS")
        identity = query(power_supply, "*IDN?")
        print("장비 정보:", identity)

        write_command(power_supply, "CONF:REM ON")
        write_command(power_supply, "CONF:OUTP OFF")

        # CC 모드를 강제하기 위해: 전압은 상한으로, 전류를 목표값으로 설정
        write_command(power_supply, f"SOUR:VOLT {VOLTAGE_CEILING}")
        check_error(power_supply)
        write_command(power_supply, f"SOUR:CURR {target_current}")
        check_error(power_supply)

        with open(csv_path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow([
                "elapsed_s", "target_current",
                "voltage_meas", "current_meas", "power_meas",
                "cv_cc_status", "temperature_c", "resistance_ohm",
            ])

            print(f"\n=== 세션 시작: {target_current} A (전압 상한 {VOLTAGE_CEILING}V) ===")
            write_command(power_supply, "CONF:OUTP ON")
            check_error(power_supply)

            session_start = time.monotonic()
            temp_history = []

            while True:
                now = time.monotonic()
                elapsed = now - session_start

                v_meas = query(power_supply, "FETC:VOLT?")
                i_meas = query(power_supply, "FETC:CURR?")
                p_meas = query(power_supply, "FETC:POW?")
                status = query(power_supply, "FETC:STAT?")

                if (not warned_cv) and elapsed > 5.0 and ",CV" in status:
                    print(
                        f"[안내] 전압 상한({VOLTAGE_CEILING}V) 안에서는 아직 목표 전류"
                        f"({target_current}A)에 도달하지 못해 CV 모드로 동작 중입니다."
                    )
                    warned_cv = True

                try:
                    if float(i_meas) > SAFETY_MAX_CURRENT:
                        print("경고: 실측 전류가 안전 상한을 초과했습니다. 즉시 출력 차단.")
                        safe_shutdown(power_supply)
                        stop_reason = "전류 안전 상한 초과 (비상 정지)"
                        break
                except ValueError:
                    pass

                temp_c = thermocouple.temperature

                try:
                    resistance = float(v_meas) / float(i_meas) if float(i_meas) != 0 else float("nan")
                except ValueError:
                    resistance = float("nan")

                print(
                    f"[{elapsed:6.1f}s] V={v_meas} I={i_meas} P={p_meas} "
                    f"STAT={status} T={temp_c:.2f}C R={resistance:.2f}ohm"
                )

                writer.writerow([
                    f"{elapsed:.1f}", target_current,
                    v_meas, i_meas, p_meas, status, f"{temp_c:.2f}", f"{resistance:.2f}",
                ])
                f.flush()

                times.append(elapsed)
                temps.append(temp_c)
                try:
                    volts.append(float(v_meas))
                except ValueError:
                    volts.append(float("nan"))

                if temp_c >= TEMP_SAFETY_CUTOFF_C:
                    stop_reason = f"온도 상한({TEMP_SAFETY_CUTOFF_C}C) 도달"
                    break

                temp_history.append((elapsed, temp_c))
                while temp_history and elapsed - temp_history[0][0] > STEADY_STATE_WINDOW_SECONDS:
                    temp_history.pop(0)
                if (
                    elapsed >= STEADY_STATE_WINDOW_SECONDS
                    and temp_history
                    and (max(t for _, t in temp_history) - min(t for _, t in temp_history)) <= STEADY_STATE_EPSILON_C
                ):
                    stop_reason = "정체(steady-state) 판정"
                    break

                if elapsed >= MAX_HOLD_SECONDS:
                    stop_reason = "시간 초과"
                    break

                time.sleep(SAMPLE_INTERVAL_SECONDS)

        print(f"=== 세션 종료 ({target_current} A) — 사유: {stop_reason} ===")
        safe_shutdown(power_supply)
        check_error(power_supply)

    except (SerialException, TimeoutError, RuntimeError) as exc:
        print(f"오류 발생: {exc}")
        stop_reason = f"오류: {exc}"
        if power_supply is not None:
            safe_shutdown(power_supply)
    finally:
        if power_supply is not None:
            power_supply.close()

    avg_temp, std_temp = compute_steady_state_average(times, temps, STEADY_STATE_WINDOW_SECONDS)
    if avg_temp is not None:
        print(f"\n정상상태(마지막 {STEADY_STATE_WINDOW_SECONDS:.0f}초) 평균 온도: {avg_temp:.2f} ± {std_temp:.2f} C")
        print("(참고: 종료 사유가 '정체(steady-state) 판정'이 아니면 이 평균은 아직 상승 중인 구간일 수 있습니다)")

    save_plot(times, temps, volts, target_current, avg_temp, std_temp, stop_reason, plot_path)
    print(f"\n결과 저장 완료: {csv_path}, {plot_path}")


if __name__ == "__main__":
    main()
