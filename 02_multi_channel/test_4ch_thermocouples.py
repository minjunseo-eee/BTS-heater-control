"""
4채널 MAX31856 연결 확인용 진단 스크립트 (v2)
- 열전대(TC) 온도뿐 아니라 칩 내장 기준접점(Cold Junction) 온도도 함께 출력
  -> CJ온도가 0이면 SPI 통신 자체가 안 되는 것 (배선/SPI 설정 문제)
  -> CJ온도는 정상인데 TC온도만 0이면 열전대 선 연결 문제 (단자/극성)
- fault 레지스터 내용도 항상 출력 (문제 원인 특정용)

사용 방법:
    python3 test_4ch_thermocouples.py
"""

import time

import board
import digitalio
import adafruit_max31856

# --- CS 핀 배치 (물리핀 기준: 29, 31, 33, 35번) ---
CS_PINS = {
    "CH1": board.D5,
    "CH2": board.D6,
    "CH3": board.D13,
    "CH4": board.D19,
}

READ_INTERVAL_SECONDS = 2.0


def main():
    spi = board.SPI()

    thermocouples = {}
    for name, pin in CS_PINS.items():
        try:
            cs = digitalio.DigitalInOut(pin)
            tc = adafruit_max31856.MAX31856(
                spi, cs, thermocouple_type=adafruit_max31856.ThermocoupleType.K
            )
            thermocouples[name] = tc
            print(f"[OK] {name} 모듈 초기화 성공 (CS={pin})")
        except Exception as e:
            print(f"[실패] {name} 모듈 초기화 실패 (CS={pin}): {e}")

    if not thermocouples:
        print("초기화된 모듈이 하나도 없습니다. 배선을 다시 확인하세요.")
        return

    print("\n--- 진단 읽기 시작 (Ctrl+C로 종료) ---")
    print("CJ = 칩 내장 기준접점 온도 (열전대 없어도 상온 근처 나와야 정상)")
    print("TC = 열전대(외부 K타입) 온도\n")

    try:
        while True:
            for name, tc in thermocouples.items():
                try:
                    cj_temp = tc.reference_temperature
                    tc_temp = tc.temperature
                    fault = tc.fault
                    active_faults = [k for k, v in fault.items() if v]
                    fault_str = ",".join(active_faults) if active_faults else "없음"
                    print(
                        f"{name}: CJ={cj_temp:6.2f}C  TC={tc_temp:6.2f}C  fault=[{fault_str}]"
                    )
                except Exception as e:
                    print(f"{name}: 읽기 실패 - {e}")
            print("-" * 60)
            time.sleep(READ_INTERVAL_SECONDS)

    except KeyboardInterrupt:
        print("\n종료합니다.")


if __name__ == "__main__":
    main()
