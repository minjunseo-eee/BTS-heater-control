# BTS 발열체 온도 제어 실험 코드

나노카본 발열체를 Chroma 62000P DC Power Supply(RS-232C 원격 제어)와
라즈베리파이 + MAX31856(K타입 열전대)로 온도 제어하는 실험 코드 모음.

## 실행 환경

- Raspberry Pi (RS-232C 시리얼로 Chroma 62000P 제어)
- Python 3
- 설치: `pip install -r requirements.txt`

## 폴더 구성

### 01_single_channel — 채널 1개 기준 실험 (초기 단계)

실험 순서대로 정리했습니다.

| 파일 | 내용 |
|---|---|
| `step1_voltage_step_test.py` | 정전압(CV) 인가 시 온도 변화 확인 |
| `step1b_current_step_test.py` | 정전류(CC) 인가 시 온도 변화 확인 |
| `analyze_step1b_results.py` | step1b 결과(CSV) 분석 스크립트 |
| `step3_fast_heat_then_drop.py` | 85도 목표, Phase1(급속승온)→Phase2(다운)→Phase3(미세조정) 3단계 제어 |
| `plot_from_step3_csv.py` | step3 CSV 결과 그래프 재생성 |

### 02_multi_channel — 채널 4개(열전대 4개) 확장 실험

| 파일 | 내용 |
|---|---|
| `test_4ch_thermocouples.py` | MAX31856 4채널 배선/통신 확인용 진단 스크립트 |
| `step4_4ch_voltage_drop.py` | 4채널 기준 최고온도 90도→전압 하강, 95도→즉시 차단 |
| `step5_4ch_phase_drop.py` | 4채널 기준 Phase1(급속승온)→Phase2(다운) 제어 |
| `step6_hysteresis_control.py` | 산업체 멘토님 스펙 기반 On/Off 히스테리시스 제어 (최종 버전, 3채널) |

> `step6_hysteresis_control.py`는 CH1 모듈을 물리적으로 제거하고 CH2~4 3채널로
> 재배선한 이후의 최종 버전입니다.

### docs — 배선도 / 제어 로직 정리 문서

| 파일 | 내용 |
|---|---|
| `4ch_max31856_wiring.html` | 4채널 MAX31856 배선도 (브라우저로 열람) |
| `pi_control_diagram.html` | 제어 로직 흐름도 (브라우저로 열람) |
| `PI_control_정리.md` | PI 제어 이론 정리 |

## 안전 한도 (공통)

- 하드웨어 절대 한도: 24V / 5A (산업체 멘토님 스펙)
- 스크립트별 온도 안전 차단 기준은 각 파일 상단 `# ===== 사용자 설정 =====` 부분 참고
