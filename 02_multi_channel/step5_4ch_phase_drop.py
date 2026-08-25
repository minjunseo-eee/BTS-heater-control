"""
Step5 — 4채널 온도 모니터링 + Phase1(급속승온) -> Phase2(다운) 전략
- step4의 4채널 읽기 + fault(단선) 처리 + 90/95도 안전장치를 그대로 이어받고,
  거기에 step3 스타일의 2단계(Phase) 전압 전략을 얹은 버전
- 평균값 계산은 하지 않음 (부위별 실측값 자체를 보는 게 목적)

동작 방식:
- Phase1: PHASE1_VOLTAGE(기본 24V)로 빠르게 승온
          4채널 중 최고값이 PHASE1_TO_2_TEMP(기본 80도)에 도달하면 Phase2로 전환
- Phase2: PHASE2_VOLTAGE(기본 16V)로 전압을 낮춰서 오버슈트 억제
- 안전장치(모든 Phase 공통, step4와 동일):
  - 최고값 90도 도달 -> 1V씩 하강 (Phase와 무관하게 항상 적용)
  - 최고값 95도 도달 -> 즉시 출력 OFF (최후 안전장치)
- fault(단선) 채널은 안전판단에서 제외 (원시값은 CSV에 남기되 경고 표시)

CSV: elapsed_s, phase, voltage_set, ch1~4_temp_c, max_temp_c, max_channel,
     voltage_meas, current_meas, power_meas
그래프: CH1~CH4 색깔별 겹쳐그리기 + Phase1/Phase2 배경 음영 구분 + 목표 85C/전환 80C/하강 90C 기준선

사용 방법:
    python3 step5_4ch_phase_drop.py            # 기본값 사용
    (전압은 스크립트 상단 PHASE1_VOLTAGE / PHASE2_VOLTAGE에서 조정)
"""

import time
import csv
from datetime import datetime

import serial

import board
import digitalio
import adafruit_max31856

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager

# ===================== 사용자 설정 =====================

PHASE1_VOLTAGE = 24.0
PHASE1_TO_2_TEMP = 80.0

PHASE2_VOLTAGE = 16.0

# 하드웨어 절대 안전 한도 (멘토 스펙 기준)
SAFETY_MAX_VOLTAGE = 24.0
SAFETY_MAX_CURRENT = 5.0
CURRENT_LIMIT = 3.0  # 보호용 전류 제한

TARGET_TEMP_C = 85.0

# 90도 도달 시 전압을 낮추는 로직 (Phase와 무관하게 항상 적용)
DROP_TRIGGER_TEMP_C = 90.0
DROP_STEP_VOLTAGE = 1.0
MIN_VOLTAGE = 0.0

# 95도 도달 시 즉시 출력 차단 (최후 안전장치)
HARD_SAFETY_CUTOFF_C = 95.0

LOG_INTERVAL_SECONDS = 2.0
MAX_TOTAL_SECONDS = 30 * 60  # 30분 세션 제한

SERIAL_PORT = "/dev/ttyUSB0"
SERIAL_BAUDRATE = 38400

# --- 4채널 CS 핀 (물리핀 기준: 29, 31, 33, 35번) ---
CS_PINS = {
    "CH1": board.D5,
    "CH2": board.D6,
    "CH3": board.D13,
    "CH4": board.D19,
}

# ===================== 한글 폰트 설정 =====================

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

_CH_COLORS = {
    "CH1": "#e53e3e",
    "CH2": "#3182ce",
    "CH3": "#38a169",
    "CH4": "#805ad5",
}

# ===================== 하드웨어 초기화 =====================


def init_thermocouples():
    spi = board.SPI()
    thermocouples = {}
    for name, pin in CS_PINS.items():
        cs = digitalio.DigitalInOut(pin)
        tc = adafruit_max31856.MAX31856(
            spi, cs, thermocouple_type=adafruit_max31856.ThermocoupleType.K
        )
        thermocouples[name] = tc
        print(f"[OK] {name} 모듈 초기화 성공 (CS={pin})")
    return thermocouples


def read_all_channels(thermocouples):
    """각 채널의 온도와 fault 상태를 함께 반환.
    open_tc(단선) 등 fault가 활성화된 채널은 temps에는 원시값을 남기되,
    faulted 세트에 표시해서 호출부에서 안전판단(최고값 계산)에 쓰지 않도록 함.
    """
    temps = {}
    faulted = set()
    for name, tc in thermocouples.items():
        try:
            temp = tc.temperature
            fault = tc.fault
            temps[name] = temp
            if any(fault.values()):
                faulted.add(name)
                active = [k for k, v in fault.items() if v]
                print(f"[경고] {name} fault 감지({','.join(active)}) - 원시값 {temp:.2f}C는 무시하고 안전판단에서 제외")
        except Exception as e:
            print(f"[경고] {name} 읽기 실패: {e}")
            temps[name] = None
            faulted.add(name)
    return temps, faulted


def send_scpi(ser, command):
    ser.write((command + "\n").encode())
    time.sleep(0.05)


def query_scpi(ser, command):
    ser.write((command + "\n").encode())
    time.sleep(0.1)
    return ser.readline().decode(errors="ignore").strip()


def set_voltage(ser, voltage):
    voltage = max(MIN_VOLTAGE, min(voltage, SAFETY_MAX_VOLTAGE))
    send_scpi(ser, f"SOUR:VOLT {voltage:.2f}")
    return voltage


# ===================== 메인 로직 =====================


def main():
    thermocouples = init_thermocouples()

    ser = serial.Serial(SERIAL_PORT, SERIAL_BAUDRATE, timeout=1)
    send_scpi(ser, "CONF:REM ON")
    send_scpi(ser, f"SOUR:CURR {CURRENT_LIMIT}")

    phase = 1
    current_voltage = set_voltage(ser, PHASE1_VOLTAGE)
    send_scpi(ser, "CONF:OUTP ON")
    print(f"[시작] Phase1, 전압 {current_voltage:.2f}V로 출력 ON (목표: 최고값 {PHASE1_TO_2_TEMP}C 도달 시 Phase2 전환)")

    timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_filename = f"step5_4ch_phase_{timestamp_str}.csv"

    rows = []
    start_time = time.monotonic()
    output_on = True

    try:
        with open(csv_filename, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(
                [
                    "elapsed_s", "phase", "voltage_set",
                    "ch1_temp_c", "ch2_temp_c", "ch3_temp_c", "ch4_temp_c",
                    "max_temp_c", "max_channel",
                    "voltage_meas", "current_meas", "power_meas",
                ]
            )

            while True:
                elapsed = time.monotonic() - start_time
                if elapsed > MAX_TOTAL_SECONDS:
                    print(f"[종료] 최대 세션 시간({MAX_TOTAL_SECONDS}s) 도달")
                    break

                temps, faulted = read_all_channels(thermocouples)
                readable_temps = {k: v for k, v in temps.items() if v is not None}
                if not readable_temps:
                    print("[오류] 모든 채널 읽기 실패, 안전을 위해 출력 차단")
                    break

                trustworthy_temps = {
                    k: v for k, v in readable_temps.items() if k not in faulted
                }

                if not trustworthy_temps:
                    print("[경고] 모든 채널이 fault 상태 - 안전판단/Phase전환 보류, 전압 유지")
                    max_channel, max_temp = None, None
                else:
                    max_channel = max(trustworthy_temps, key=trustworthy_temps.get)
                    max_temp = trustworthy_temps[max_channel]

                # --- 최후 안전장치: 95도 도달 시 즉시 차단 ---
                if max_temp is not None and max_temp >= HARD_SAFETY_CUTOFF_C:
                    print(
                        f"[안전차단] {max_channel} {max_temp:.2f}C >= "
                        f"{HARD_SAFETY_CUTOFF_C}C, 즉시 출력 OFF"
                    )
                    send_scpi(ser, "CONF:OUTP OFF")
                    output_on = False
                    rows.append([
                        elapsed, phase, current_voltage,
                        *[temps.get(f"CH{i}") for i in range(1, 5)],
                        max_temp, max_channel, None, None, None,
                    ])
                    writer.writerow(rows[-1])
                    break

                # --- Phase1 -> Phase2 전환 ---
                if phase == 1 and max_temp is not None and max_temp >= PHASE1_TO_2_TEMP:
                    phase = 2
                    current_voltage = set_voltage(ser, PHASE2_VOLTAGE)
                    print(
                        f"[Phase전환] {max_channel} {max_temp:.2f}C >= {PHASE1_TO_2_TEMP}C "
                        f"-> Phase2 진입, 전압 {current_voltage:.2f}V로 하강"
                    )

                # --- 90도 도달 시 1V씩 하강 (Phase 무관, 항상 적용) ---
                if max_temp is not None and max_temp >= DROP_TRIGGER_TEMP_C and current_voltage > MIN_VOLTAGE:
                    new_voltage = max(MIN_VOLTAGE, current_voltage - DROP_STEP_VOLTAGE)
                    current_voltage = set_voltage(ser, new_voltage)
                    print(
                        f"[전압하강] {max_channel} {max_temp:.2f}C >= {DROP_TRIGGER_TEMP_C}C "
                        f"-> 전압 {current_voltage:.2f}V로 하강"
                    )

                v_meas = query_scpi(ser, "FETC:VOLT?")
                c_meas = query_scpi(ser, "FETC:CURR?")
                p_meas = query_scpi(ser, "FETC:POW?")

                def to_float(s):
                    try:
                        return float(s)
                    except ValueError:
                        return None

                v_meas_f = to_float(v_meas)
                c_meas_f = to_float(c_meas)
                p_meas_f = to_float(p_meas)

                row = [
                    round(elapsed, 2), phase, current_voltage,
                    temps.get("CH1"), temps.get("CH2"), temps.get("CH3"), temps.get("CH4"),
                    max_temp, max_channel,
                    v_meas_f, c_meas_f, p_meas_f,
                ]
                rows.append(row)
                writer.writerow(row)

                def fmt_ch(k):
                    if temps.get(k) is None:
                        return f"{k}=X"
                    marker = "!" if k in faulted else ""
                    return f"{k}={temps[k]:.2f}C{marker}"

                temp_str = " | ".join(fmt_ch(k) for k in ["CH1", "CH2", "CH3", "CH4"])
                max_str = f"{max_channel}({max_temp:.2f}C)" if max_temp is not None else "판단보류(전채널fault)"
                print(
                    f"[{elapsed:6.1f}s][Phase{phase}] {temp_str} | 최고={max_str} "
                    f"| V설정={current_voltage:.2f} | V실측={v_meas} | I실측={c_meas}"
                )

                time.sleep(LOG_INTERVAL_SECONDS)

    except KeyboardInterrupt:
        print("\n[중단] 사용자가 Ctrl+C로 중단함")
    finally:
        if output_on:
            send_scpi(ser, "CONF:OUTP OFF")
            print("[안전] 출력 OFF 처리 완료")
        ser.close()

    print(f"\nCSV 저장 완료: {csv_filename}")
    save_plot(rows, csv_filename)


def save_plot(rows, csv_filename):
    if not rows:
        print("[안내] 기록된 데이터가 없어 그래프를 생성하지 않습니다.")
        return

    times = [r[0] for r in rows]
    phases = [r[1] for r in rows]
    ch_temps = {
        "CH1": [r[3] for r in rows],
        "CH2": [r[4] for r in rows],
        "CH3": [r[5] for r in rows],
        "CH4": [r[6] for r in rows],
    }
    volts = [r[2] for r in rows]

    fig, ax1 = plt.subplots(figsize=(11, 6))

    # Phase1 / Phase2 배경 음영 (전환 시점을 axvspan으로 표시)
    phase2_start_idx = next((i for i, p in enumerate(phases) if p == 2), None)
    if phase2_start_idx is not None:
        ax1.axvspan(times[0], times[phase2_start_idx], color="#fed7d7", alpha=0.3,
                    label="Phase1" if KOREAN_FONT_AVAILABLE else "Phase1")
        ax1.axvspan(times[phase2_start_idx], times[-1], color="#bee3f8", alpha=0.3,
                    label="Phase2" if KOREAN_FONT_AVAILABLE else "Phase2")

    for name, temps in ch_temps.items():
        ax1.plot(times, temps, color=_CH_COLORS[name], linewidth=1.4, label=name)

    ax1.axhline(TARGET_TEMP_C, color="gray", linestyle=":", linewidth=1,
                label="목표 85C" if KOREAN_FONT_AVAILABLE else "Target 85C")
    ax1.axhline(PHASE1_TO_2_TEMP, color="#c05621", linestyle="--", linewidth=1,
                label="Phase전환 80C" if KOREAN_FONT_AVAILABLE else "Phase switch 80C")
    ax1.axhline(DROP_TRIGGER_TEMP_C, color="orange", linestyle="--", linewidth=1,
                label="전압하강 90C" if KOREAN_FONT_AVAILABLE else "Drop trigger 90C")

    if KOREAN_FONT_AVAILABLE:
        ax1.set_xlabel("경과 시간 (s)")
        ax1.set_ylabel("온도 (C)")
        title = "4채널 온도 모니터링 — Phase1(급속승온) -> Phase2(다운)"
    else:
        ax1.set_xlabel("Elapsed time (s)")
        ax1.set_ylabel("Temperature (C)")
        title = "4-channel monitoring — Phase1 (fast heat) -> Phase2 (drop)"

    ax1.set_title(title)
    ax1.grid(True, alpha=0.3)

    ax2 = ax1.twinx()
    ax2.plot(times, volts, color="black", linewidth=0.8, alpha=0.5, linestyle="--",
              label="전압" if KOREAN_FONT_AVAILABLE else "Voltage")
    ax2.set_ylabel("전압 (V)" if KOREAN_FONT_AVAILABLE else "Voltage (V)")

    lines1, labels1 = ax1.get_legend_handles_labels()
    lines2, labels2 = ax2.get_legend_handles_labels()
    ax1.legend(lines1 + lines2, labels1 + labels2, loc="lower right", fontsize=8)

    fig.tight_layout()
    out_path = csv_filename.rsplit(".", 1)[0] + ".png"
    fig.savefig(out_path, dpi=150)
    plt.close(fig)
    print(f"그래프 저장 완료: {out_path}")


if __name__ == "__main__":
    main()
