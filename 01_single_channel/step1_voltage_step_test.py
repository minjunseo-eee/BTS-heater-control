"""
1단계 실험: 정전압 응답 테스트 (한 세션 = 한 전압)
- Chroma 62000P DC Power Supply를 RS-232C(SCPI)로 제어하며 전압 하나를 인가
- MAX31856 + K타입 열전대로 온도를 동시에 로깅
- 세션 종료 시 CSV 저장 + 온도-시간 그래프(PNG) 자동 생성 + 정상상태(saturation) 평균 온도 계산

사용 방법:
    python3 step1_voltage_step_test.py 12.0
    (인자로 전압값 하나를 넘김. 생략하면 아래 VOLTAGE 기본값 사용)

한 세션 = 한 전압입니다. 다음 전압을 테스트하기 전에는 발열체가 충분히
식은 걸 직접 눈으로 확인한 뒤 스크립트를 다시 실행하세요 (자동 냉각 대기 없음).

사용 전 반드시 확인:
1) Chroma 62000P 본체 Baud Rate가 38400으로 설정되어 있는지
2) MAX31856 배선 (SPI: SCK/MOSI/MISO + CS 핀) 및 라이브러리 설치
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
matplotlib.use("Agg")  # 화면 없는 라즈베리파이 환경에서도 저장 가능하도록
import matplotlib.pyplot as plt
from matplotlib import font_manager

# --- 한글 폰트 설정 ---
# 라즈베리파이 기본 matplotlib에는 한글 폰트가 없어서 그대로 두면 글자가 깨짐(네모 박스).
# 아래 후보 중 설치된 폰트를 자동으로 찾아서 적용하고, 하나도 없으면 영문 라벨로 대체함.
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
        "       한글로 보고 싶으면 라즈베리파이에서 아래 명령 실행 후 다시 시도하세요:\n"
        "       sudo apt-get install -y fonts-nanum\n"
        "       rm -rf ~/.cache/matplotlib"
    )

# ============================================================
# 설정값 (필요에 맞게 수정)
# ============================================================

# --- 이번 세션에서 테스트할 전압 (커맨드라인 인자로 덮어쓸 수 있음) ---
VOLTAGE = 12.0

# --- Chroma 62000P 통신 설정 ---
PORT = "/dev/ttyUSB0"
BAUD_RATE = 38400          # 본체 설정과 반드시 일치해야 함

# --- 안전 상한 (하드웨어 스펙: 24V / 5A) ---
SAFETY_MAX_VOLTAGE = 24.0
SAFETY_MAX_CURRENT = 5.0

# --- 이번 실험에서 쓸 전류 제한 (보호용, CC 진입 목적 아님) ---
CURRENT_LIMIT = 3.0

MAX_HOLD_SECONDS = 10 * 60          # 세션 최대 유지 시간 (초)
SAMPLE_INTERVAL_SECONDS = 2.0       # 로깅 주기

# --- 온도 안전 상한 (도달 즉시 세션 종료 + 출력 차단) ---
TEMP_SAFETY_CUTOFF_C = 85.0

# --- 정체(steady-state) 감지: 최근 WINDOW초 동안 온도 변화가 EPSILON도 이내면 조기 종료 ---
STEADY_STATE_WINDOW_SECONDS = 90.0
STEADY_STATE_EPSILON_C = 0.3

# --- MAX31856 SPI 설정 ---
CS_PIN = board.D5          # 실제 결선한 CS 핀으로 변경
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
# 결과 처리: 정상상태 평균 계산 + 그래프 생성
# ============================================================

def compute_steady_state_average(times, temps, window_s: float):
    """마지막 window_s 구간의 평균/표준편차를 계산. (해당 구간이 실제로 정체였는지는 호출부에서 stop_reason으로 판단)"""
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


def save_plot(times, temps, voltage: float, avg_temp, std_temp, stop_reason: str, out_path: str) -> None:
    if KOREAN_FONT_AVAILABLE:
        xlabel, ylabel = "경과 시간 (s)", "온도 (C)"
        series_label = "온도 실측"
        avg_label = f"정상상태 평균 {avg_temp:.2f}±{std_temp:.2f}C" if avg_temp is not None else ""
        title = f"{voltage}V 정전압 응답 — 종료 사유: {stop_reason}"
    else:
        xlabel, ylabel = "Elapsed time (s)", "Temperature (C)"
        series_label = "Measured temperature"
        avg_label = f"Steady-state avg {avg_temp:.2f}±{std_temp:.2f}C" if avg_temp is not None else ""
        title = f"{voltage}V constant-voltage response — stop reason: {_STOP_REASON_EN.get(stop_reason, stop_reason)}"

    fig, ax = plt.subplots(figsize=(9, 5))
    ax.plot(times, temps, color="#2b6cb0", linewidth=1.2, label=series_label)

    if avg_temp is not None:
        ax.axhline(
            avg_temp, color="#c53030", linestyle="--", linewidth=1,
            label=avg_label,
        )

    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
    ax.set_title(title)
    ax.legend(loc="lower right")
    ax.grid(True, alpha=0.3)

    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)


# ============================================================
# 메인 실험 루틴
# ============================================================

def main() -> None:
    voltage = VOLTAGE
    if len(sys.argv) > 1:
        voltage = float(sys.argv[1])

    if voltage > SAFETY_MAX_VOLTAGE:
        raise ValueError(f"전압({voltage}V)이 안전 상한(24V)을 초과합니다.")
    if CURRENT_LIMIT > SAFETY_MAX_CURRENT:
        raise ValueError("전류 제한값이 안전 상한(5A)을 초과합니다.")

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_path = f"step1_{voltage:g}V_{timestamp}.csv"
    plot_path = f"step1_{voltage:g}V_{timestamp}.png"

    # --- 온도 센서 초기화 ---
    spi = board.SPI()
    cs = digitalio.DigitalInOut(CS_PIN)
    thermocouple = adafruit_max31856.MAX31856(
        spi, cs, thermocouple_type=THERMOCOUPLE_TYPE
    )

    power_supply: Optional[serial.Serial] = None
    times, temps = [], []
    stop_reason = "시간 초과"

    try:
        power_supply = serial.Serial(
            port=PORT,
            baudrate=BAUD_RATE,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=2.0,
            write_timeout=2.0,
            xonxoff=False,
            rtscts=False,
            dsrdtr=False,
        )
        power_supply.reset_input_buffer()
        power_supply.reset_output_buffer()

        write_command(power_supply, "*CLS")
        identity = query(power_supply, "*IDN?")
        print("장비 정보:", identity)

        write_command(power_supply, "CONF:REM ON")
        write_command(power_supply, "CONF:OUTP OFF")

        write_command(power_supply, f"SOUR:CURR {CURRENT_LIMIT}")
        check_error(power_supply)

        with open(csv_path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow([
                "elapsed_s", "voltage_set",
                "voltage_meas", "current_meas", "power_meas",
                "cv_cc_status", "temperature_c", "resistance_ohm",
            ])

            print(f"\n=== 세션 시작: {voltage} V ===")
            write_command(power_supply, f"SOUR:VOLT {voltage}")
            check_error(power_supply)
            write_command(power_supply, "CONF:OUTP ON")
            check_error(power_supply)

            session_start = time.monotonic()
            temp_history = []  # (elapsed, temp), 정체 판정용

            while True:
                now = time.monotonic()
                elapsed = now - session_start

                v_meas = query(power_supply, "FETC:VOLT?")
                i_meas = query(power_supply, "FETC:CURR?")
                p_meas = query(power_supply, "FETC:POW?")
                status = query(power_supply, "FETC:STAT?")

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
                    f"{elapsed:.1f}", voltage,
                    v_meas, i_meas, p_meas, status, f"{temp_c:.2f}", f"{resistance:.2f}",
                ])
                f.flush()

                times.append(elapsed)
                temps.append(temp_c)

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

        print(f"=== 세션 종료 ({voltage} V) — 사유: {stop_reason} ===")
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

    # --- 정상상태 평균 계산 + 그래프 저장 ---
    avg_temp, std_temp = compute_steady_state_average(times, temps, STEADY_STATE_WINDOW_SECONDS)
    if avg_temp is not None:
        print(f"\n정상상태(마지막 {STEADY_STATE_WINDOW_SECONDS:.0f}초) 평균 온도: {avg_temp:.2f} ± {std_temp:.2f} C")
        print(f"(참고: 종료 사유가 '정체(steady-state) 판정'이 아니면 이 평균은 아직 상승 중인 구간일 수 있습니다)")

    save_plot(times, temps, voltage, avg_temp, std_temp, stop_reason, plot_path)
    print(f"\n결과 저장 완료: {csv_path}, {plot_path}")


if __name__ == "__main__":
    main()
