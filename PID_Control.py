"""
PID 기반 온도 제어 — 4채널, 목표온도 78도 안착
- 14V/16V 정전압 테스트 실측 데이터로 계산한 게인 사용 (SIMC 튜닝법)
  실측: K(공정이득) ~= 5 C/V, tau(시정수) ~= 150s, L(지연) ~= 5s (거의 없음)
  -> Cohen-Coon은 지연이 거의 없는 이 시스템엔 게인이 비정상적으로 커져서(Kc~8) 부적합.
     대신 SIMC(Skogestad) 방식으로 lambda(목표 폐루프 시정수) = tau/3 로 계산:
       Kp = tau / (K*(lambda+L))
       Ti = tau            -> Ki = Kp / Ti
       Td = L/2            -> Kd = Kp * Td

동작 방식:
- 피드백 기준: 4채널 중 최고값(max) 채널 -> 안전 마진 확보 (CH3/CH4가 항상 제일 뜨거웠음)
- 매 루프마다 실제 경과시간(dt)을 측정해서 적분/미분 계산 (고정 2초 가정 X)
  -> 시리얼 응답 지연으로 루프 주기가 늘어져도 PID 적분값이 왜곡되지 않도록 함
- 출력(전압)은 0~VOLTAGE_MAX(기본 20V)로 클램프
- 안티와인드업: 출력이 상/하한에 물려있고 오차 방향이 그 쪽으로 더 미는 상황이면 적분 누적 정지
- 안전장치는 PID 위에 별도 레이어로 항상 적용 (PID가 뭐라고 계산하든 최종 강제):
  - 최고값 90도 이상 -> PID 출력과 무관하게 전압을 1V씩 강제로 더 낮춤
  - 최고값 95도 이상 -> 즉시 출력 OFF
  - 전채널 fault -> 온도 감시 불가로 간주, 즉시 출력 OFF

CSV: elapsed_s, dt_s, ch1~4_temp_c, max_temp_c, max_channel, error_c,
     p_term, i_term, d_term, voltage_command, voltage_meas, current_meas, power_meas

그래프: 위 - CH1~4 온도 + 목표선, 아래 - 인가 전압(실측)

사용 방법:
    python3 step_pid_control.py
    (목표온도/게인/전압상한은 아래 사용자 설정에서 조정)
"""

import os
import time
import csv
import threading
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

TARGET_TEMP_C = 78.0  # 목표(정상상태) 온도

# --- PID 게인 (14V/16V 실측 데이터 기반 SIMC 튜닝) ---
# K~=5 C/V, tau~=150s, L~=5s, lambda=tau/3=50s 로 계산한 값.
# 실제 하드웨어 반응 보고 필요시 미세조정 가능.
KP = 0.55
KI = KP / 150.0   # Ti = 150s
KD = KP * 2.5     # Td = 2.5s (지연이 작아 비중 작게)

# 전압 한계 — 20V에서 SPI 통신이 자꾸 멈추는 현상이 있어 임시로 18V로 낮춰서
# 테스트 중 (16V에서는 예전에 완전히 문제없이 끝까지 돌았던 기록 있음).
# 절연/접지 쪽 원인이 확실히 해결되면 다시 20V로 올려도 됨.
VOLTAGE_MAX = 18.0
VOLTAGE_MIN = 0.0

# 적분 누적 한도 (안티와인드업) — 적분항(Ki*integral) 혼자서 낼 수 있는 전압이
# 최대 VOLTAGE_MAX를 넘지 않도록 역산해서 정함. 이 값을 임의로 작게 잡으면
# (예: 40처럼 감으로 정하면) 정상상태에 필요한 전압을 적분항이 못 만들어내서
# 목표온도에 영원히 도달 못 하는 정상상태 오차가 생김 — 시뮬레이션으로 확인된 버그였음.
INTEGRAL_LIMIT = VOLTAGE_MAX / KI
CURRENT_LIMIT = 3.0  # 보호용 전류 제한

# 슬루레이트 제한 (안정성 진단용 추가) — PID가 계산한 목표전압으로 한 번에
# 점프하지 않고, 한 루프(LOG_INTERVAL_SECONDS)당 최대 이만큼만 전압을 바꾼다.
# 목적: Chroma 출력이 급격히 바뀔 때 생기는 스위칭 과도현상/노이즈가
# SPI(.temperature)나 시리얼(FETC:VOLT? 등) 통신을 멈추게 하는 것으로 의심되어,
# 전압 변화를 완만하게 만들어 노이즈 유입 자체를 줄여보려는 시도.
MAX_VOLTAGE_STEP_PER_LOOP = 0.5

# 90도 도달 시 PID와 무관하게 강제로 전압을 낮추는 로직 (여전히 유지)
DROP_TRIGGER_TEMP_C = 90.0
DROP_STEP_VOLTAGE = 1.0

# 95도 도달 시 즉시 출력 차단 (최후 안전장치)
HARD_SAFETY_CUTOFF_C = 95.0

LOG_INTERVAL_SECONDS = 2.0  # 목표 주기 (실제 dt는 매번 다시 측정해서 사용)
MAX_TOTAL_SECONDS = 45 * 60  # 45분 세션 제한

# 노이즈로 인해 SPI 통신이 멈추는 것 외에, 가끔 완전히 잘못된 값(글리치)이
# 튀는 것도 실측으로 확인됨 (예: 4초 만에 25도 상승 - 물리적으로 불가능).
# 시정수(~150s) 기준으로 이 이상 빠른 변화는 노이즈로 간주하고 버림.
MAX_PLAUSIBLE_RATE_C_PER_S = 3.0

# MAX31856이 fault(단선 등) 상태일 때 .temperature가 반환하는 값은 1372C 부근의
# 고정 에러 센티널이다. 이 값을 실제 온도로 착각해서 그래프에 그리면 y축이
# 1372C까지 늘어나면서 정작 보고 싶은 0~100C 구간 곡선이 바닥에 납작하게 눌린다.
# -> save_plot()에서 이 값을 넘는 데이터는 그래프상 빈 구간(None)으로 처리한다.
FAULT_CEILING_C = 150.0

# 고전압 인가 중 SPI 통신이 멈추는 현상이 실측으로 확인됨 (EMI/접지 노이즈 의심).
# 메인 루프가 이 시간(초) 동안 한 번도 진행되지 않으면, 별도 스레드가 무조건
# 강제로 출력을 끄고 프로세스를 종료한다 — 메인 루프가 멈춰도 안전장치(90/95도
# 체크)까지 같이 멈춰버리는 걸 막기 위한 최후의 방어선.
WATCHDOG_TIMEOUT_SECONDS = 15.0

DEBUG = True  # True면 각 단계마다 어디까지 진행됐는지 출력 (멈추는 지점 찾는 용도)

SERIAL_PORT = "/dev/ttyUSB0"
SERIAL_BAUDRATE = 38400

# --- 4채널 CS 핀 ---
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
    temps = {}
    faulted = set()
    for name, tc in thermocouples.items():
        try:
            if DEBUG:
                print(f"[디버그]   {name} .temperature 읽기 시작", flush=True)
            temp = tc.temperature
            if DEBUG:
                print(f"[디버그]   {name} .temperature={temp:.2f} 완료, .fault 읽기 시작", flush=True)
            fault = tc.fault
            if DEBUG:
                print(f"[디버그]   {name} .fault 완료", flush=True)
            temps[name] = temp
            if any(fault.values()):
                faulted.add(name)
                active = [k for k, v in fault.items() if v]
                print(f"[경고] {name} fault 감지({','.join(active)}) - 안전판단에서 제외")
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
    voltage = max(VOLTAGE_MIN, min(voltage, VOLTAGE_MAX))
    send_scpi(ser, f"SOUR:VOLT {voltage:.2f}")
    return voltage


# ===================== PID 컨트롤러 =====================


class PID:
    def __init__(self, kp, ki, kd, integral_limit):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral_limit = integral_limit
        self.integral = 0.0
        self.prev_measurement = None

    def compute(self, setpoint, measurement, dt, output_saturated_high, output_saturated_low):
        error = setpoint - measurement

        # --- 안티와인드업: 출력이 포화된 방향으로 오차가 더 미는 중이면 적분 정지 ---
        would_wind_further = (output_saturated_high and error > 0) or (
            output_saturated_low and error < 0
        )
        if not would_wind_further and dt > 0:
            self.integral += error * dt
            self.integral = max(-self.integral_limit, min(self.integral_limit, self.integral))

        # --- 미분항은 오차가 아니라 "측정값"에 대해 계산 (setpoint가 고정이라 큰 차이는 없지만
        #     혹시 목표온도를 바꾸는 경우 미분 킥(derivative kick)을 방지하기 위한 표준 관례) ---
        if self.prev_measurement is None or dt <= 0:
            derivative = 0.0
        else:
            derivative = -(measurement - self.prev_measurement) / dt
        self.prev_measurement = measurement

        p_term = self.kp * error
        i_term = self.ki * self.integral
        d_term = self.kd * derivative

        output = p_term + i_term + d_term
        return output, error, p_term, i_term, d_term


# ===================== 워치독 (메인 루프 행 감시) =====================


def watchdog_thread_func(heartbeat, stop_event):
    """메인 루프가 SPI 등에서 멈춰도 무조건 출력을 끄는 독립 감시 스레드.
    메인 스레드와 별개의 시리얼 연결을 직접 열어서 사용 — 메인 스레드가
    어디서 멈추든(SPI, 다른 I/O 등) 이 스레드는 영향받지 않고 계속 동작한다."""
    try:
        wd_ser = serial.Serial(SERIAL_PORT, SERIAL_BAUDRATE, timeout=1)
    except Exception as e:
        print(f"[워치독] 감시용 시리얼 연결 실패({e}) - 워치독 비활성화 상태로 진행됩니다", flush=True)
        return

    try:
        while not stop_event.is_set():
            time.sleep(1.0)
            idle = time.monotonic() - heartbeat["t"]
            if idle > WATCHDOG_TIMEOUT_SECONDS:
                print(
                    f"\n[워치독-비상] 메인 루프가 {idle:.1f}초 동안 응답이 없습니다! "
                    f"(SPI 통신 멈춤 등으로 안전장치가 같이 멈췄을 가능성) "
                    f"독립적으로 강제 출력 OFF 처리합니다.",
                    flush=True,
                )
                for _ in range(3):
                    try:
                        wd_ser.write(b"CONF:OUTP OFF\n")
                        wd_ser.flush()
                    except Exception as e:
                        print(f"[워치독] 출력 OFF 명령 전송 실패: {e}", flush=True)
                    time.sleep(0.2)
                print(
                    "[워치독-비상] 출력 OFF 명령 전송 완료. "
                    "메인 루프가 멈춘 상태라 정상 종료가 불가능해 프로세스를 강제 종료합니다.",
                    flush=True,
                )
                try:
                    wd_ser.close()
                except Exception:
                    pass
                os._exit(1)
    finally:
        try:
            wd_ser.close()
        except Exception:
            pass


# ===================== 메인 로직 =====================


def main():
    thermocouples = init_thermocouples()

    ser = serial.Serial(SERIAL_PORT, SERIAL_BAUDRATE, timeout=1)
    send_scpi(ser, "CONF:REM ON")
    send_scpi(ser, f"SOUR:CURR {CURRENT_LIMIT}")

    current_voltage = set_voltage(ser, VOLTAGE_MIN)
    send_scpi(ser, "CONF:OUTP ON")

    pid = PID(KP, KI, KD, INTEGRAL_LIMIT)

    heartbeat = {"t": time.monotonic()}
    stop_event = threading.Event()
    watchdog = threading.Thread(
        target=watchdog_thread_func, args=(heartbeat, stop_event), daemon=True
    )
    watchdog.start()
    print(f"[워치독] 감시 시작 (메인 루프가 {WATCHDOG_TIMEOUT_SECONDS:.0f}초 이상 멈추면 강제 출력 OFF)")

    print(f"[시작] PID 제어 시작, 목표 {TARGET_TEMP_C:.1f}C (max채널 기준), 전압상한 {VOLTAGE_MAX}V")
    print(f"[게인] Kp={KP:.4f} Ki={KI:.6f} Kd={KD:.4f}\n")

    timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_filename = f"step_pid_{timestamp_str}.csv"

    rows = []
    start_time = time.monotonic()
    last_time = start_time
    output_on = True

    # 채널별 "마지막으로 믿을 수 있었던" 온도/시각 -- 글리치 필터링용
    last_valid_temp = {}
    last_valid_time = {}

    try:
        with open(csv_filename, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(
                [
                    "elapsed_s", "dt_s",
                    "ch1_temp_c", "ch2_temp_c", "ch3_temp_c", "ch4_temp_c",
                    "max_temp_c", "max_channel", "error_c",
                    "p_term", "i_term", "d_term",
                    "voltage_command", "voltage_meas", "current_meas", "power_meas",
                ]
            )

            while True:
                now = time.monotonic()
                elapsed = now - start_time
                dt = now - last_time
                last_time = now
                heartbeat["t"] = now  # 워치독에게 "메인 루프 살아있음" 신호

                if elapsed > MAX_TOTAL_SECONDS:
                    print(f"[종료] 최대 세션 시간({MAX_TOTAL_SECONDS}s) 도달")
                    break

                if DEBUG:
                    print("[디버그] 루프 시작, 온도 읽기 전", flush=True)

                temps, faulted = read_all_channels(thermocouples)

                if DEBUG:
                    print(f"[디버그] 온도 읽기 완료: {temps}", flush=True)

                # --- 글리치(물리적으로 불가능한 급변) 필터링 ---
                # 노이즈로 인해 순간적으로 튄 값을 걸러내고, 대신 직전 신뢰값을 사용.
                # (진짜 단선/fault는 이 필터와 별개로 faulted 세트에서 계속 잡아냄)
                for name in list(temps.keys()):
                    val = temps.get(name)
                    if val is None or name in faulted:
                        continue
                    prev_val = last_valid_temp.get(name)
                    prev_t = last_valid_time.get(name)
                    if prev_val is not None and prev_t is not None:
                        dt_ch = now - prev_t
                        if dt_ch > 0:
                            rate = abs(val - prev_val) / dt_ch
                            if rate > MAX_PLAUSIBLE_RATE_C_PER_S:
                                print(
                                    f"[경고] {name} 글리치 의심(변화율 {rate:.2f}C/s > "
                                    f"{MAX_PLAUSIBLE_RATE_C_PER_S}C/s) - 이번 값 버리고 "
                                    f"직전값 {prev_val:.2f}C 유지",
                                    flush=True,
                                )
                                temps[name] = prev_val
                                continue
                    last_valid_temp[name] = val
                    last_valid_time[name] = now

                readable_temps = {k: v for k, v in temps.items() if v is not None}
                if not readable_temps:
                    print("[비상차단] 모든 채널 읽기 실패, 안전을 위해 출력 차단")
                    send_scpi(ser, "CONF:OUTP OFF")
                    output_on = False
                    break

                trustworthy_temps = {
                    k: v for k, v in readable_temps.items() if k not in faulted
                }

                if not trustworthy_temps:
                    print("[비상차단] 모든 채널이 fault 상태 - 온도 감시 불가능, 즉시 출력 OFF")
                    send_scpi(ser, "CONF:OUTP OFF")
                    output_on = False
                    rows.append([
                        elapsed, dt,
                        *[temps.get(f"CH{i}") for i in range(1, 5)],
                        None, None, None, None, None, None, None, None, None, None,
                    ])
                    writer.writerow(rows[-1])
                    break
                else:
                    max_channel = max(trustworthy_temps, key=trustworthy_temps.get)
                    max_temp = trustworthy_temps[max_channel]

                # --- 최후 안전장치: 95도 도달 시 즉시 차단 ---
                if max_temp >= HARD_SAFETY_CUTOFF_C:
                    print(f"[안전차단] {max_channel} {max_temp:.2f}C >= {HARD_SAFETY_CUTOFF_C}C, 즉시 출력 OFF")
                    send_scpi(ser, "CONF:OUTP OFF")
                    output_on = False
                    rows.append([
                        elapsed, dt,
                        *[temps.get(f"CH{i}") for i in range(1, 5)],
                        max_temp, max_channel, None, None, None, None,
                        current_voltage, None, None, None,
                    ])
                    writer.writerow(rows[-1])
                    break

                # --- PID 계산 ---
                if DEBUG:
                    print("[디버그] PID 계산 전", flush=True)

                output_saturated_high = current_voltage >= VOLTAGE_MAX
                output_saturated_low = current_voltage <= VOLTAGE_MIN
                pid_output, error, p_term, i_term, d_term = pid.compute(
                    TARGET_TEMP_C, max_temp, dt, output_saturated_high, output_saturated_low
                )

                if DEBUG:
                    print(f"[디버그] PID 계산 완료: output={pid_output:.2f}, 전압설정 시작", flush=True)

                # --- 슬루레이트 제한: 목표전압(pid_output)으로 한 번에 점프하지 않고
                #     이번 루프에서는 현재전압 기준 +-MAX_VOLTAGE_STEP_PER_LOOP 까지만 이동 ---
                voltage_step = max(
                    -MAX_VOLTAGE_STEP_PER_LOOP,
                    min(MAX_VOLTAGE_STEP_PER_LOOP, pid_output - current_voltage),
                )
                target_voltage = current_voltage + voltage_step
                current_voltage = set_voltage(ser, target_voltage)

                if DEBUG:
                    print(
                        f"[디버그] 전압설정 완료: {current_voltage:.2f}V "
                        f"(PID목표={pid_output:.2f}V, 이번스텝={voltage_step:+.2f}V)",
                        flush=True,
                    )

                # --- 90도 도달 시 PID와 무관하게 강제로 1V씩 더 하강 (이중 안전장치) ---
                if max_temp >= DROP_TRIGGER_TEMP_C and current_voltage > VOLTAGE_MIN:
                    forced_voltage = max(VOLTAGE_MIN, current_voltage - DROP_STEP_VOLTAGE)
                    current_voltage = set_voltage(ser, forced_voltage)
                    print(f"[강제하강] {max_channel} {max_temp:.2f}C >= {DROP_TRIGGER_TEMP_C}C -> 전압 {current_voltage:.2f}V로 강제 조정")

                if DEBUG:
                    print("[디버그] V실측 쿼리 시작", flush=True)

                v_meas_raw = query_scpi(ser, "FETC:VOLT?")

                if DEBUG:
                    print(f"[디버그] V실측 완료: {v_meas_raw!r}, I실측 쿼리 시작", flush=True)

                c_meas_raw = query_scpi(ser, "FETC:CURR?")

                if DEBUG:
                    print(f"[디버그] I실측 완료: {c_meas_raw!r}, P실측 쿼리 시작", flush=True)

                p_meas_raw = query_scpi(ser, "FETC:POW?")

                if DEBUG:
                    print(f"[디버그] P실측 완료: {p_meas_raw!r}", flush=True)

                def to_float(s):
                    try:
                        return float(s)
                    except ValueError:
                        return None

                row = [
                    round(elapsed, 2), round(dt, 3),
                    temps.get("CH1"), temps.get("CH2"), temps.get("CH3"), temps.get("CH4"),
                    max_temp, max_channel, error,
                    p_term, i_term, d_term,
                    current_voltage, to_float(v_meas_raw), to_float(c_meas_raw), to_float(p_meas_raw),
                ]
                rows.append(row)
                writer.writerow(row)

                def fmt_ch(k):
                    if temps.get(k) is None:
                        return f"{k}=X"
                    marker = "!" if k in faulted else ""
                    return f"{k}={temps[k]:.2f}C{marker}"

                temp_str = " | ".join(fmt_ch(k) for k in ["CH1", "CH2", "CH3", "CH4"])
                print(
                    f"[{elapsed:6.1f}s] {temp_str} | 최고={max_channel}({max_temp:.2f}C) "
                    f"| 오차={error:+.2f}C | V명령={current_voltage:.2f} "
                    f"(P={p_term:+.2f} I={i_term:+.2f} D={d_term:+.2f})",
                    flush=True,
                )

                sleep_time = max(0.0, LOG_INTERVAL_SECONDS - (time.monotonic() - now))

                if DEBUG:
                    print(f"[디버그] 이번 루프 소요시간={time.monotonic()-now:.2f}s, {sleep_time:.2f}초 대기 시작", flush=True)

                time.sleep(sleep_time)

    except KeyboardInterrupt:
        print("\n[중단] 사용자가 Ctrl+C로 중단함")
    finally:
        stop_event.set()  # 워치독 스레드 정상 종료
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
    volts = [r[12] for r in rows]

    def clean(v):
        """fault 센티널 값(1372C 부근)을 실제 데이터가 아닌 빈 구간(None)으로 변환.
        None이 들어가면 matplotlib이 그 지점에서 선을 끊어 그리므로,
        fault였던 구간이 마치 정상 값처럼 이어지지 않는다."""
        if v is None:
            return None
        if v > FAULT_CEILING_C:
            return None
        return v

    # CSV 컬럼 순서: 0 elapsed_s, 1 dt_s, 2 ch1, 3 ch2, 4 ch3, 5 ch4,
    # 6 max_temp_c, 7 max_channel, 8 error_c, 9 p_term, 10 i_term, 11 d_term,
    # 12 voltage_command, 13 voltage_meas, 14 current_meas, 15 power_meas
    ch_temps = {
        "CH1": [clean(r[2]) for r in rows],
        "CH2": [clean(r[3]) for r in rows],
        "CH3": [clean(r[4]) for r in rows],
        "CH4": [clean(r[5]) for r in rows],
    }

    fig, (ax1, ax2) = plt.subplots(
        2, 1, figsize=(11, 7), sharex=True,
        gridspec_kw={"height_ratios": [3, 1]},
    )

    for name, temps in ch_temps.items():
        ax1.plot(times, temps, color=_CH_COLORS[name], linewidth=1.4, label=name)

    ax1.axhline(TARGET_TEMP_C, color="gray", linestyle=":", linewidth=1.2,
                label=f"목표 {TARGET_TEMP_C:.0f}C" if KOREAN_FONT_AVAILABLE else f"Target {TARGET_TEMP_C:.0f}C")
    ax1.axhline(DROP_TRIGGER_TEMP_C, color="orange", linestyle="--", linewidth=1,
                label="강제하강 90C" if KOREAN_FONT_AVAILABLE else "Forced drop 90C")
    ax1.axhline(HARD_SAFETY_CUTOFF_C, color="#c53030", linestyle="--", linewidth=1,
                label="안전차단 95C" if KOREAN_FONT_AVAILABLE else "Hard cutoff 95C")

    # y축을 fault 값에 끌려가지 않도록 실제 관심 구간(0~105C)으로 고정
    ax1.set_ylim(0, max(HARD_SAFETY_CUTOFF_C + 10, 100))

    if KOREAN_FONT_AVAILABLE:
        ax1.set_ylabel("온도 (C)")
        ax1.set_title(f"PID 온도 제어 — 목표 {TARGET_TEMP_C:.0f}C (max채널 기준)")
        ax2.set_xlabel("경과 시간 (s)")
        ax2.set_ylabel("전압 명령 (V)")
    else:
        ax1.set_ylabel("Temperature (C)")
        ax1.set_title(f"PID temperature control — target {TARGET_TEMP_C:.0f}C (max channel)")
        ax2.set_xlabel("Elapsed time (s)")
        ax2.set_ylabel("Voltage command (V)")

    ax1.grid(True, alpha=0.3)
    ax1.legend(loc="lower right", fontsize=8)

    ax2.plot(times, volts, color="black", linewidth=1.2)
    ax2.grid(True, alpha=0.3)

    fig.tight_layout()
    out_path = csv_filename.rsplit(".", 1)[0] + ".png"
    fig.savefig(out_path, dpi=150)
    plt.close(fig)
    print(f"그래프 저장 완료: {out_path}")


if __name__ == "__main__":
    main()
