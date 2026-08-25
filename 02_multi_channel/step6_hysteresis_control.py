"""
Step6 — On/Off 히스테리시스(Hysteresis) 제어 (멘토님 스펙 기반)
- 지금까지의 step4/step5(목표온도 정밀 수렴형)와는 완전히 다른 방식.
  전압을 세밀하게 조절하지 않고 완전 ON/OFF만 함.
- 부위별 온도 편차가 커서, 판단 기준은 평균이 아니라 3채널 중 "최고값"으로 함
  -> 최고값이 OFF_TEMP_C(기본 80도) 이상이면 즉시 출력 OFF
  -> 그 상태에서 최고값이 ON_TEMP_C(기본 70도) 이하로 내려가면 다시 출력 ON
- fault(단선) 채널은 최고값 판단에서 제외 (step4/step5와 동일한 방식)
- ON 상태의 전압은 ON_VOLTAGE(기본 17V, CV모드, 이전에 검증된 값)로 고정
- 최후 안전장치: 80도 히스테리시스가 정상 작동했다면 절대 도달할 일이 없는
  HARD_SAFETY_CUTOFF_C(기본 90도)에 도달하면 무조건 즉시 차단 (이상 상황 대비)
- 평균온도도 참고용으로 CSV/그래프에 같이 남기지만, 제어 판단에는 쓰지 않음
- LCD 표시 기능은 이번 버전에서는 제외 (보류)

CSV: elapsed_s, output_state(ON/OFF), avg_temp_c(참고용), n_channels_used,
     ch1~4_temp_c, voltage_meas, current_meas, power_meas
그래프: 최고값선(굵게, 실제 판단 기준) + 평균선(참고용, 얇게) + CH2~4(옅게)
        + 70/80도 기준선 + 전압(ON/OFF 계단모양, 보조축)

사용 방법:
    python3 step6_hysteresis_control.py
    (전압/온도 기준은 스크립트 상단에서 조정)
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

ON_VOLTAGE = 18.0  # 켜졌을 때 인가할 전압 (CV모드) — 이전 검증된 값 사용

OFF_TEMP_C = 80.0   # 이 온도 이상이면 OFF
ON_TEMP_C = 70.0    # 이 온도 이하로 내려가면 다시 ON

# 하드웨어 절대 안전 한도 (멘토 스펙 기준)
SAFETY_MAX_VOLTAGE = 24.0
SAFETY_MAX_CURRENT = 5.0
CURRENT_LIMIT = 3.0  # 보호용 전류 제한

# 최후 안전장치: 80도 기준(아래 최고값 히스테리시스)으로 이미 꺼졌어야 하는데
# 못 꺼진 이상 상황을 잡기 위한 진짜 최후 안전장치. 무조건 즉시 차단.
HARD_SAFETY_CUTOFF_C = 90.0

LOG_INTERVAL_SECONDS = 2.0
MAX_TOTAL_SECONDS = 30 * 60  # 30분 세션 제한

SERIAL_PORT = "/dev/ttyUSB0"
SERIAL_BAUDRATE = 38400

# --- 3채널 CS 핀 (CH1 모듈 제거 - 공유버스는 파이->CH2로 직결 배선했다고 가정) ---
# (물리핀 기준: CH2=31번, CH3=33번, CH4=35번)
CS_PINS = {
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
                print(f"[경고] {name} fault 감지({','.join(active)}) - 평균/안전판단에서 제외")
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


# ===================== 메인 로직 =====================


def main():
    thermocouples = init_thermocouples()

    ser = serial.Serial(SERIAL_PORT, SERIAL_BAUDRATE, timeout=1)
    send_scpi(ser, "CONF:REM ON")
    send_scpi(ser, f"SOUR:CURR {CURRENT_LIMIT}")
    send_scpi(ser, f"SOUR:VOLT {ON_VOLTAGE:.2f}")

    output_state = "ON"
    send_scpi(ser, "CONF:OUTP ON")
    print(f"[시작] 출력 ON ({ON_VOLTAGE}V) — 평균 {OFF_TEMP_C}C 도달 시 OFF, {ON_TEMP_C}C 이하 시 다시 ON")

    timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_filename = f"step6_hysteresis_{timestamp_str}.csv"

    rows = []
    start_time = time.monotonic()

    try:
        with open(csv_filename, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(
                [
                    "elapsed_s", "output_state",
                    "max_temp_c", "max_channel",
                    "avg_temp_c", "n_channels_used",
                    "ch2_temp_c", "ch3_temp_c", "ch4_temp_c",
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
                    send_scpi(ser, "CONF:OUTP OFF")
                    output_state = "OFF"
                    break

                trustworthy_temps = {
                    k: v for k, v in readable_temps.items() if k not in faulted
                }

                if trustworthy_temps:
                    avg_temp = sum(trustworthy_temps.values()) / len(trustworthy_temps)
                    n_used = len(trustworthy_temps)
                    max_channel = max(trustworthy_temps, key=trustworthy_temps.get)
                    max_temp = trustworthy_temps[max_channel]
                else:
                    avg_temp, n_used = None, 0
                    max_channel, max_temp = None, None
                    print("[경고] 모든 채널이 fault 상태 - 판단 보류, 현재 상태 유지")

                # --- 최후 안전장치: 개별 채널 중 최고값이 90도 넘으면 무조건 차단 ---
                # (아래 히스테리시스가 80도에서 이미 껐어야 하므로, 여기 도달하는 건 이상 상황)
                if max_temp is not None and max_temp >= HARD_SAFETY_CUTOFF_C:
                    print(
                        f"[안전차단] {max_channel} {max_temp:.2f}C >= "
                        f"{HARD_SAFETY_CUTOFF_C}C, 즉시 출력 OFF (히스테리시스 로직 무시)"
                    )
                    send_scpi(ser, "CONF:OUTP OFF")
                    output_state = "OFF"
                    rows.append([
                        elapsed, output_state, max_temp, max_channel, avg_temp, n_used,
                        *[temps.get(f"CH{i}") for i in range(2, 5)],
                        None, None, None,
                    ])
                    writer.writerow(rows[-1])
                    break

                # --- 히스테리시스 On/Off 제어 (부위별 편차가 커서 "최고값" 기준으로 판단) ---
                if max_temp is not None:
                    if output_state == "ON" and max_temp >= OFF_TEMP_C:
                        send_scpi(ser, "CONF:OUTP OFF")
                        output_state = "OFF"
                        print(f"[OFF] {max_channel} 최고온도 {max_temp:.2f}C >= {OFF_TEMP_C}C")
                    elif output_state == "OFF" and max_temp <= ON_TEMP_C:
                        send_scpi(ser, f"SOUR:VOLT {ON_VOLTAGE:.2f}")
                        send_scpi(ser, "CONF:OUTP ON")
                        output_state = "ON"
                        print(f"[ON] {max_channel} 최고온도 {max_temp:.2f}C <= {ON_TEMP_C}C")

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
                    round(elapsed, 2), output_state, max_temp, max_channel, avg_temp, n_used,
                    temps.get("CH2"), temps.get("CH3"), temps.get("CH4"),
                    v_meas_f, c_meas_f, p_meas_f,
                ]
                rows.append(row)
                writer.writerow(row)

                def fmt_ch(k):
                    if temps.get(k) is None:
                        return f"{k}=X"
                    marker = "!" if k in faulted else ""
                    return f"{k}={temps[k]:.2f}C{marker}"

                temp_str = " | ".join(fmt_ch(k) for k in ["CH2", "CH3", "CH4"])
                max_str = f"{max_channel}({max_temp:.2f}C)" if max_temp is not None else "판단보류"
                avg_str = f"{avg_temp:.2f}C" if avg_temp is not None else "N/A"
                print(
                    f"[{elapsed:6.1f}s][{output_state}] {temp_str} | 최고(판단기준)={max_str} "
                    f"| 평균(참고)={avg_str} | V실측={v_meas} | I실측={c_meas}"
                )

                time.sleep(LOG_INTERVAL_SECONDS)

    except KeyboardInterrupt:
        print("\n[중단] 사용자가 Ctrl+C로 중단함")
        send_scpi(ser, "CONF:OUTP OFF")
        print("[안전] 출력 OFF 처리 완료")
    else:
        send_scpi(ser, "CONF:OUTP OFF")
        print("[안전] 출력 OFF 처리 완료")
    finally:
        ser.close()

    print(f"\nCSV 저장 완료: {csv_filename}")
    save_plot(rows, csv_filename)


def save_plot(rows, csv_filename):
    if not rows:
        print("[안내] 기록된 데이터가 없어 그래프를 생성하지 않습니다.")
        return

    times = [r[0] for r in rows]
    states = [r[1] for r in rows]
    max_temps = [r[2] for r in rows]
    avg_temps = [r[4] for r in rows]
    ch_temps = {
        "CH2": [r[6] for r in rows],
        "CH3": [r[7] for r in rows],
        "CH4": [r[8] for r in rows],
    }
    volt_step = [ON_VOLTAGE if s == "ON" else 0.0 for s in states]

    fig, ax1 = plt.subplots(figsize=(11, 6))

    for name, temps in ch_temps.items():
        ax1.plot(times, temps, color=_CH_COLORS[name], linewidth=1.1, alpha=0.45, label=name)

    ax1.plot(times, avg_temps, color="#a0aec0", linewidth=1.3, linestyle="-.",
              label="평균(참고)" if KOREAN_FONT_AVAILABLE else "Average (reference)")

    ax1.plot(times, max_temps, color="#1a202c", linewidth=2.2,
              label="최고값(판단기준)" if KOREAN_FONT_AVAILABLE else "Max (control basis)")

    ax1.axhline(OFF_TEMP_C, color="#c53030", linestyle="--", linewidth=1,
                label=f"OFF {OFF_TEMP_C}C" if KOREAN_FONT_AVAILABLE else f"OFF {OFF_TEMP_C}C")
    ax1.axhline(ON_TEMP_C, color="#2b6cb0", linestyle="--", linewidth=1,
                label=f"ON {ON_TEMP_C}C" if KOREAN_FONT_AVAILABLE else f"ON {ON_TEMP_C}C")

    if KOREAN_FONT_AVAILABLE:
        ax1.set_xlabel("경과 시간 (s)")
        ax1.set_ylabel("온도 (C)")
        title = f"On/Off 히스테리시스 제어 ({ON_TEMP_C}C~{OFF_TEMP_C}C)"
    else:
        ax1.set_xlabel("Elapsed time (s)")
        ax1.set_ylabel("Temperature (C)")
        title = f"On/Off hysteresis control ({ON_TEMP_C}C~{OFF_TEMP_C}C)"

    ax1.set_title(title)
    ax1.grid(True, alpha=0.3)

    ax2 = ax1.twinx()
    ax2.step(times, volt_step, where="post", color="black", linewidth=0.9, alpha=0.5,
              linestyle="--", label="전압(ON/OFF)" if KOREAN_FONT_AVAILABLE else "Voltage (ON/OFF)")
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
