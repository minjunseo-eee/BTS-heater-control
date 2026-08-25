"""
Step4 — 4채널 온도 모니터링 + 90도 도달 시 전압 자동 하강
- 시작 전압은 커맨드라인 인자로 받되, 17V 초과는 사용하지 않는 것을 전제로 함
  (17V 초과 입력 시 경고만 출력, 강제 차단은 아님 — 필요하면 SAFETY_MAX_VOLTAGE로 하드 제한)
- CH1~CH4 열전대 중 최고값(어느 채널이든)을 기준으로 안전 판단
  - 최고값 90도 도달 -> 1V씩 하강 (즉시 차단 아님)
  - 최고값 95도 도달 -> 최후 안전장치로 즉시 출력 OFF (하드 컷오프)
- 평균값 계산: fault(단선 등) 채널만 제외하고 나머지를 단순 평균
  (부위별 온도 차이는 발열체 위치 특성상 실제로 클 수 있는 정상 데이터라
   중앙값 기반 이상치 제외는 하지 않음 - fault 제외만이 유일한 필터)
  - 매 시점 평균 계산에 실제로 몇 개 채널이 쓰였는지(n_channels_used) 같이 기록
  - 살아있는(non-fault) 채널이 하나도 없으면 평균은 빈 값으로 남기고 억지로 채우지 않음
- 안전판단(90도/95도 트리거)은 fault만 제외한 채널들의 최고값을 그대로 사용
- CSV: 시간, 4채널 온도, 평균온도, 평균에 쓰인 채널 수, 설정전압, 실측전압/전류/전력 기록
- 그래프: CH1~CH4를 색깔 다르게 겹쳐서 표시 + 평균선(굵게 강조) + 목표 85도 기준선 + 전압 트레이스(보조축)

사용 방법:
    python3 step4_4ch_voltage_drop.py 15      # 15V로 시작
    python3 step4_4ch_voltage_drop.py         # 기본값(DEFAULT_START_VOLTAGE) 사용
"""

import sys
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

DEFAULT_START_VOLTAGE = 18.0

# 소프트 상한(경고만) — 이번 실험에서는 17V를 넘기지 않을 계획
SOFT_MAX_VOLTAGE = 19.0

# 하드웨어 절대 안전 한도 (멘토 스펙 기준)
SAFETY_MAX_VOLTAGE = 24.0
SAFETY_MAX_CURRENT = 5.0
CURRENT_LIMIT = 3.0  # 보호용 전류 제한 (CC 진입 목적 아님)

TARGET_TEMP_C = 85.0

# 90도 도달 시 전압을 낮추는 로직
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
    -> MAX31856은 열전대가 끊기면(open circuit) K타입 표현범위 최댓값인
       1372도 근처의 값을 반환하는데, 이건 실제 온도가 아니라 에러 신호임.
    """
    temps = {}
    faulted = set()
    for name, tc in thermocouples.items():
        try:
            temp = tc.temperature
            fault = tc.fault
            temps[name] = temp
            if fault.get("open_tc") or any(fault.values()):
                faulted.add(name)
                active = [k for k, v in fault.items() if v]
                print(f"[경고] {name} fault 감지({','.join(active)}) - 원시값 {temp:.2f}C는 무시하고 안전판단에서 제외")
        except Exception as e:
            print(f"[경고] {name} 읽기 실패: {e}")
            temps[name] = None
            faulted.add(name)
    return temps, faulted


def compute_reliable_average(trustworthy_temps):
    """fault(단선 등) 채널만 걸러낸 상태에서 단순 평균 계산.
    부위별 온도 차이는 발열체 위치 특성상 실제로 클 수 있어(정상 데이터),
    중앙값 기반 이상치 제외는 하지 않음 -> fault 제외만이 유일한 필터.
    반환: (평균 또는 None, 실제 사용된 채널 수, 빈 리스트[호환용])
    """
    if not trustworthy_temps:
        return None, 0, []

    values = list(trustworthy_temps.values())
    avg = sum(values) / len(values)
    return avg, len(values), []


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
    start_voltage = DEFAULT_START_VOLTAGE
    if len(sys.argv) > 1:
        try:
            start_voltage = float(sys.argv[1])
        except ValueError:
            print(f"[오류] 전압 인자를 숫자로 해석할 수 없습니다: {sys.argv[1]}")
            return

    if start_voltage > SOFT_MAX_VOLTAGE:
        print(
            f"[경고] 시작 전압 {start_voltage}V가 계획한 상한({SOFT_MAX_VOLTAGE}V)을 "
            f"넘습니다. 의도한 값이 맞는지 확인하세요. (하드웨어 절대 한도: {SAFETY_MAX_VOLTAGE}V)"
        )

    thermocouples = init_thermocouples()

    ser = serial.Serial(SERIAL_PORT, SERIAL_BAUDRATE, timeout=1)
    send_scpi(ser, "CONF:REM ON")
    send_scpi(ser, f"SOUR:CURR {CURRENT_LIMIT}")

    current_voltage = set_voltage(ser, start_voltage)
    send_scpi(ser, "CONF:OUTP ON")
    print(f"[시작] 초기 전압 {current_voltage:.2f}V로 출력 ON")

    timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_filename = f"step4_4ch_{start_voltage:g}V_{timestamp_str}.csv"

    rows = []
    start_time = time.monotonic()
    output_on = True

    try:
        with open(csv_filename, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(
                [
                    "elapsed_s", "voltage_set",
                    "ch1_temp_c", "ch2_temp_c", "ch3_temp_c", "ch4_temp_c",
                    "max_temp_c", "max_channel",
                    "avg_temp_c", "n_channels_used", "outlier_channels",
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

                # fault(단선 등)가 감지된 채널은 안전판단(최고값)에서 제외
                trustworthy_temps = {
                    k: v for k, v in readable_temps.items() if k not in faulted
                }

                if not trustworthy_temps:
                    # 전 채널이 fault 상태 -> 진짜 온도를 알 수 없으므로
                    # 전압을 더 올리거나 내리지 않고 현재 상태 유지, 경고만 출력
                    print("[경고] 모든 채널이 fault 상태 - 안전판단 보류, 전압 유지")
                    max_channel, max_temp = None, None
                else:
                    max_channel = max(trustworthy_temps, key=trustworthy_temps.get)
                    max_temp = trustworthy_temps[max_channel]

                # --- 신뢰성 있는 평균 계산 (fault 제외 + 이상치 제외) ---
                avg_temp, n_used, outlier_channels = compute_reliable_average(trustworthy_temps)
                if outlier_channels:
                    print(f"[경고] 이상치로 평균에서 제외된 채널: {outlier_channels}")

                # --- 최후 안전장치: 95도 도달 시 즉시 차단 (신뢰 가능한 채널이 있을 때만 판단) ---
                if max_temp is not None and max_temp >= HARD_SAFETY_CUTOFF_C:
                    print(
                        f"[안전차단] {max_channel} {max_temp:.2f}C >= "
                        f"{HARD_SAFETY_CUTOFF_C}C, 즉시 출력 OFF"
                    )
                    send_scpi(ser, "CONF:OUTP OFF")
                    output_on = False
                    rows.append([
                        elapsed, current_voltage,
                        *[temps.get(f"CH{i}") for i in range(1, 5)],
                        max_temp, max_channel,
                        avg_temp, n_used, ";".join(outlier_channels),
                        None, None, None,
                    ])
                    writer.writerow(rows[-1])
                    break

                # --- 90도 도달 시 1V씩 하강 (신뢰 가능한 채널이 있을 때만 판단) ---
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
                    round(elapsed, 2), current_voltage,
                    temps.get("CH1"), temps.get("CH2"), temps.get("CH3"), temps.get("CH4"),
                    max_temp, max_channel,
                    avg_temp, n_used, ";".join(outlier_channels),
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
                avg_str = f"{avg_temp:.2f}C(n={n_used})" if avg_temp is not None else "없음"
                print(
                    f"[{elapsed:6.1f}s] {temp_str} | 최고={max_str} | 평균={avg_str} "
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
    ch_temps = {
        "CH1": [r[2] for r in rows],
        "CH2": [r[3] for r in rows],
        "CH3": [r[4] for r in rows],
        "CH4": [r[5] for r in rows],
    }
    volts = [r[1] for r in rows]
    avg_temps = [r[8] for r in rows]

    fig, ax1 = plt.subplots(figsize=(11, 6))

    for name, temps in ch_temps.items():
        # None 값은 선이 끊기도록 그대로 두고 matplotlib이 처리하게 함
        ax1.plot(times, temps, color=_CH_COLORS[name], linewidth=1.2, alpha=0.55, label=name)

    ax1.plot(times, avg_temps, color="#1a202c", linewidth=2.4,
              label="평균(fault제외)" if KOREAN_FONT_AVAILABLE else "Average (fault excluded)")

    ax1.axhline(TARGET_TEMP_C, color="gray", linestyle=":", linewidth=1,
                label="목표 85C" if KOREAN_FONT_AVAILABLE else "Target 85C")
    ax1.axhline(DROP_TRIGGER_TEMP_C, color="orange", linestyle="--", linewidth=1,
                label="전압하강 90C" if KOREAN_FONT_AVAILABLE else "Drop trigger 90C")

    if KOREAN_FONT_AVAILABLE:
        ax1.set_xlabel("경과 시간 (s)")
        ax1.set_ylabel("온도 (C)")
        title = "4채널 온도 모니터링 + 90C 도달 시 전압 하강"
    else:
        ax1.set_xlabel("Elapsed time (s)")
        ax1.set_ylabel("Temperature (C)")
        title = "4-channel monitoring with voltage drop at 90C"

    ax1.set_title(title)
    ax1.grid(True, alpha=0.3)

    ax2 = ax1.twinx()
    ax2.plot(times, volts, color="black", linewidth=0.8, alpha=0.5, linestyle="--",
              label="전압" if KOREAN_FONT_AVAILABLE else "Voltage")
    ax2.set_ylabel("전압 (V)" if KOREAN_FONT_AVAILABLE else "Voltage (V)")

    lines1, labels1 = ax1.get_legend_handles_labels()
    lines2, labels2 = ax2.get_legend_handles_labels()
    ax1.legend(lines1 + lines2, labels1 + labels2, loc="lower right", fontsize=9)

    fig.tight_layout()
    out_path = csv_filename.rsplit(".", 1)[0] + ".png"
    fig.savefig(out_path, dpi=150)
    plt.close(fig)
    print(f"그래프 저장 완료: {out_path}")


if __name__ == "__main__":
    main()
