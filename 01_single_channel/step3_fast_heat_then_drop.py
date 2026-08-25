"""
3단계 실험: 초기 고전압 급속승온 후 다운시키는 전략 (교수님 방향성)
- Phase 1: 24V로 급속 승온
- Phase 2: 80도 도달 시 15V로 다운 (관성으로 더 오르는 걸 감안해 목표보다 낮은 온도에서 미리 전환)
  15V에서 온도가 정체(steady-state, 60초/2.5도 기준)로 판정되면 Phase 3로 전환
  (혹시 정체 판정이 계속 안 나도 8분 지나면 강제로 Phase3 전환하는 백업 타임아웃 포함)
- Phase 3 (수렴 단계): 15V -> 16V로 먼저 한 번 점프, 이후로는 CHECK_INTERVAL_SECONDS마다
  목표(85도) 기준으로 비교해서:
    - 목표보다 낮으면 FINE_STEP_VOLTAGE 만큼 전압 증가
    - 목표보다 높으면 FINE_STEP_VOLTAGE 만큼 전압 감소
    - 허용오차(TARGET_TOLERANCE_C) 안에 들어오면 그대로 유지
  이런 식으로 85도 근처로 서서히 수렴시킴 (초기 형태의 폐루프 제어)
- 하드 안전장치: 어떤 상황이든 90도를 넘으면 즉시 출력 차단 (미세조정이 못 따라갈 경우 최종 방어선)
- MAX31856 + K타입 열전대로 온도를 동시에 로깅
- 세션 종료 시 CSV 저장 + Phase별로 색이 구분된 온도/전압 그래프(PNG) 자동 생성

사용 방법:
    python3 step3_fast_heat_then_drop.py

사용 전 반드시 확인:
1) Chroma 62000P 본체 Baud Rate가 38400으로 설정되어 있는지
2) MAX31856 배선 및 라이브러리 설치
     pip install adafruit-circuitpython-max31856 --break-system-packages
     pip install matplotlib --break-system-packages
3) PORT 경로 (ls /dev/ttyUSB*)
4) 전압/전류 상한 (24V / 5A) — 코드 내 SAFETY LIMIT 밖의 값은 절대 넣지 말 것
"""

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

PORT = "/dev/ttyUSB0"
BAUD_RATE = 38400

SAFETY_MAX_VOLTAGE = 24.0
SAFETY_MAX_CURRENT = 5.0
CURRENT_LIMIT = 3.0                 # 보호용 전류 상한

# --- Phase 전압 값 ---
PHASE1_VOLTAGE = 24.0               # 급속승온
PHASE2_VOLTAGE = 15.0               # 다운 (안전하게 식힘)
PHASE3_JUMP_VOLTAGE = 16.0          # Phase3 진입 시 첫 점프 전압

# --- Phase 전환 기준 온도 ---
PHASE1_TO_2_TEMP = 80.0             # 이 온도 도달 시 24V -> 15V

# --- Phase2 -> Phase3 전환: 15V에서 온도 정체(steady-state) 판정 ---
STEADY_STATE_WINDOW_SECONDS = 40.0
STEADY_STATE_EPSILON_C = 2.5        # 실측 데이터 기준으로 설정 (창을 40초로 줄였으니 필요시 epsilon도 같이 재조정)
PHASE2_MAX_HOLD_SECONDS = 8 * 60     # 정체 판정이 계속 안 나도 8분 지나면 강제로 Phase3 전환 (백업 타임아웃)

# --- Phase 3 수렴 로직: 목표 온도 기준으로 위/아래 미세 조정 ---
TARGET_TEMP_C = 85.0
TARGET_TOLERANCE_C = 0.5            # 84.5~85.5 안이면 그대로 유지
FINE_STEP_VOLTAGE = 0.1
FINE_TUNE_CHECK_INTERVAL_SECONDS = 30.0

# --- 하드 안전장치: 이 온도 넘으면 무조건 즉시 차단 ---
HARD_SAFETY_CUTOFF_C = 90.0

MAX_TOTAL_SECONDS = 40 * 60          # 전체 세션 최대 시간 (초). 필요시 조정
SAMPLE_INTERVAL_SECONDS = 2.0

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


def set_voltage(device: serial.Serial, voltage: float) -> None:
    voltage = max(0.0, min(voltage, SAFETY_MAX_VOLTAGE))
    write_command(device, f"SOUR:VOLT {voltage:.2f}")


# ============================================================
# 그래프 생성
# ============================================================

_PHASE_COLORS = {
    1: "#e53e3e",   # 급속승온 - 빨강
    2: "#3182ce",   # 다운 - 파랑
    3: "#805ad5",   # 수렴(미세조정) - 보라
}
_PHASE_LABELS_KO = {
    1: "Phase1 급속승온(24V)",
    2: "Phase2 다운(15V)",
    3: "Phase3 수렴(16V 점프 + 0.1V 미세조정)",
}
_PHASE_LABELS_EN = {
    1: "Phase1 fast heat (24V)",
    2: "Phase2 drop (15V)",
    3: "Phase3 converge (16V jump + 0.1V fine-tune)",
}


def save_plot(times, temps, volts, phases, out_path):
    labels = _PHASE_LABELS_KO if KOREAN_FONT_AVAILABLE else _PHASE_LABELS_EN

    fig, ax1 = plt.subplots(figsize=(11, 6))

    # phase별로 구간을 나눠서 색을 다르게 그림
    seen_labels = set()
    start_idx = 0
    for i in range(1, len(phases) + 1):
        if i == len(phases) or phases[i] != phases[start_idx]:
            phase = phases[start_idx]
            seg_times = times[start_idx:i]
            seg_temps = temps[start_idx:i]
            label = labels[phase] if phase not in seen_labels else None
            ax1.plot(seg_times, seg_temps, color=_PHASE_COLORS[phase], linewidth=1.5, label=label)
            seen_labels.add(phase)
            start_idx = i

    if KOREAN_FONT_AVAILABLE:
        ax1.axhline(85.0, color="gray", linestyle=":", linewidth=1, label="목표 85C")
        ax1.set_xlabel("경과 시간 (s)")
        ax1.set_ylabel("온도 (C)", color="black")
        title = "초기 고전압 급속승온 후 다운 전략 (Phase별 결과)"
    else:
        ax1.axhline(85.0, color="gray", linestyle=":", linewidth=1, label="Target 85C")
        ax1.set_xlabel("Elapsed time (s)")
        ax1.set_ylabel("Temperature (C)", color="black")
        title = "Fast-heat-then-drop strategy (by phase)"

    ax1.grid(True, alpha=0.3)

    ax2 = ax1.twinx()
    ax2.plot(times, volts, color="black", linewidth=0.8, alpha=0.5, linestyle="--")
    ylabel_v = "전압 (V)" if KOREAN_FONT_AVAILABLE else "Voltage (V)"
    ax2.set_ylabel(ylabel_v, color="black")

    ax1.set_title(title)
    ax1.legend(loc="lower right", fontsize=9)

    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)


# ============================================================
# 메인 실험 루틴
# ============================================================

def main() -> None:
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_path = f"step3_fastheat_{timestamp}.csv"
    plot_path = f"step3_fastheat_{timestamp}.png"

    spi = board.SPI()
    cs = digitalio.DigitalInOut(CS_PIN)
    thermocouple = adafruit_max31856.MAX31856(spi, cs, thermocouple_type=THERMOCOUPLE_TYPE)

    power_supply: Optional[serial.Serial] = None
    times, temps, volts, phases = [], [], [], []

    current_phase = 1
    current_voltage = PHASE1_VOLTAGE
    last_fine_tune_time = None
    phase2_temp_history = []  # (elapsed, temp), Phase2 정체 판정용
    phase2_start_elapsed = None

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
        write_command(power_supply, f"SOUR:CURR {CURRENT_LIMIT}")
        check_error(power_supply)

        print(f"\n=== Phase 1 시작: {PHASE1_VOLTAGE}V (급속승온) ===")
        set_voltage(power_supply, current_voltage)
        check_error(power_supply)
        write_command(power_supply, "CONF:OUTP ON")
        check_error(power_supply)

        with open(csv_path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow([
                "elapsed_s", "phase", "voltage_set",
                "voltage_meas", "current_meas", "power_meas",
                "cv_cc_status", "temperature_c",
            ])

            session_start = time.monotonic()

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
                        break
                except ValueError:
                    pass

                temp_c = thermocouple.temperature

                # --- 하드 안전장치: 최우선으로 체크 ---
                if temp_c >= HARD_SAFETY_CUTOFF_C:
                    print(f"[비상] 온도가 하드 안전 상한({HARD_SAFETY_CUTOFF_C}C)을 초과했습니다. 즉시 출력 차단.")
                    safe_shutdown(power_supply)
                    times.append(elapsed); temps.append(temp_c); volts.append(current_voltage); phases.append(current_phase)
                    writer.writerow([f"{elapsed:.1f}", current_phase, current_voltage, v_meas, i_meas, p_meas, status, f"{temp_c:.2f}"])
                    break

                # --- Phase 전환 로직 ---
                if current_phase == 1 and temp_c >= PHASE1_TO_2_TEMP:
                    current_phase = 2
                    current_voltage = PHASE2_VOLTAGE
                    print(f"\n=== Phase 2 전환: {PHASE2_VOLTAGE}V (다운, 온도 {temp_c:.1f}C 도달) ===")
                    set_voltage(power_supply, current_voltage)
                    check_error(power_supply)
                    phase2_temp_history = []
                    phase2_start_elapsed = elapsed

                elif current_phase == 2:
                    # 정체(steady-state) 판정: 최근 WINDOW초 동안 변화가 EPSILON 이내면 Phase3로 전환
                    phase2_temp_history.append((elapsed, temp_c))
                    while phase2_temp_history and elapsed - phase2_temp_history[0][0] > STEADY_STATE_WINDOW_SECONDS:
                        phase2_temp_history.pop(0)
                    # 주의: while문이 "WINDOW초 넘는 샘플은 pop"하는 구조라 남은 이력의 span은
                    # 항상 WINDOW 이하로 유지됨. 그래서 "span >= WINDOW"로 비교하면 사실상 절대 만족되지
                    # 않는 버그가 생김. 대신 "Phase2 진입 후 WINDOW초 이상 지났는지"로 판단해야 함.
                    phase2_elapsed = elapsed - phase2_start_elapsed if phase2_start_elapsed is not None else 0.0

                    steady_detected = (
                        phase2_elapsed >= STEADY_STATE_WINDOW_SECONDS
                        and (max(t for _, t in phase2_temp_history) - min(t for _, t in phase2_temp_history)) <= STEADY_STATE_EPSILON_C
                    )
                    timeout_fallback = phase2_elapsed >= PHASE2_MAX_HOLD_SECONDS

                    if steady_detected or timeout_fallback:
                        current_phase = 3
                        current_voltage = PHASE3_JUMP_VOLTAGE
                        reason = "15V 정체 판정" if steady_detected else f"{PHASE2_MAX_HOLD_SECONDS/60:.0f}분 타임아웃(정체 판정 대신 강제 전환)"
                        print(f"\n=== Phase 3 전환: {PHASE3_JUMP_VOLTAGE}V로 점프 ({reason}, 온도 {temp_c:.1f}C) ===")
                        set_voltage(power_supply, current_voltage)
                        check_error(power_supply)
                        last_fine_tune_time = elapsed

                elif current_phase == 3:
                    # 목표(85도) 기준 위/아래로 CHECK_INTERVAL마다 미세 조정
                    if last_fine_tune_time is None or (elapsed - last_fine_tune_time) >= FINE_TUNE_CHECK_INTERVAL_SECONDS:
                        if temp_c < TARGET_TEMP_C - TARGET_TOLERANCE_C:
                            current_voltage = min(SAFETY_MAX_VOLTAGE, current_voltage + FINE_STEP_VOLTAGE)
                            print(f"  [미세조정] 목표보다 낮음({temp_c:.2f}C) -> 전압 {current_voltage:.2f}V로 상향")
                            set_voltage(power_supply, current_voltage)
                            check_error(power_supply)
                        elif temp_c > TARGET_TEMP_C + TARGET_TOLERANCE_C:
                            current_voltage = max(0.0, current_voltage - FINE_STEP_VOLTAGE)
                            print(f"  [미세조정] 목표보다 높음({temp_c:.2f}C) -> 전압 {current_voltage:.2f}V로 하향")
                            set_voltage(power_supply, current_voltage)
                            check_error(power_supply)
                        else:
                            print(f"  [미세조정] 허용오차 이내({temp_c:.2f}C) -> {current_voltage:.2f}V 유지")
                        last_fine_tune_time = elapsed

                print(
                    f"[{elapsed:6.1f}s] Phase{current_phase} V_set={current_voltage:.2f} "
                    f"V={v_meas} I={i_meas} P={p_meas} STAT={status} T={temp_c:.2f}C"
                )

                writer.writerow([
                    f"{elapsed:.1f}", current_phase, f"{current_voltage:.2f}",
                    v_meas, i_meas, p_meas, status, f"{temp_c:.2f}",
                ])
                f.flush()

                times.append(elapsed)
                temps.append(temp_c)
                volts.append(current_voltage)
                phases.append(current_phase)

                if elapsed >= MAX_TOTAL_SECONDS:
                    print("=== 최대 실행시간 도달, 세션 종료 ===")
                    break

                time.sleep(SAMPLE_INTERVAL_SECONDS)

        print("\n실험 종료. 출력 OFF 처리.")
        safe_shutdown(power_supply)
        check_error(power_supply)

    except (SerialException, TimeoutError, RuntimeError) as exc:
        print(f"오류 발생: {exc}")
        if power_supply is not None:
            safe_shutdown(power_supply)
    finally:
        if power_supply is not None:
            power_supply.close()

    if times:
        save_plot(times, temps, volts, phases, plot_path)
        print(f"\n결과 저장 완료: {csv_path}, {plot_path}")


if __name__ == "__main__":
    main()
