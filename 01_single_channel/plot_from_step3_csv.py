"""
Step3(초기 고전압 급속승온 후 다운 전략) 결과 CSV로부터 그래프만 다시 그리는 스크립트
- step3_fast_heat_then_drop.py 실행 중 CSV는 남았지만 그래프가 없거나(중간에 멈춘 경우 등),
  다시 그리고 싶을 때 사용

사용 방법:
    python3 plot_from_step3_csv.py step3_fastheat_20260819_153000.csv
    (인자 없이 실행하면 아래 DEFAULT_CSV_FILE을 사용)
"""

import sys
import csv

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager

# --- 커맨드라인 인자 없이 실행할 때 쓰일 기본 파일. 실제 파일명으로 바꿔서 쓰세요 ---
DEFAULT_CSV_FILE = "step3_fastheat_20260819_153000.csv"

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


def load_csv(path):
    times, temps, volts, phases = [], [], [], []
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                times.append(float(row["elapsed_s"]))
                temps.append(float(row["temperature_c"]))
                volts.append(float(row["voltage_set"]))
                phases.append(int(float(row["phase"])))
            except (ValueError, KeyError):
                continue
    return times, temps, volts, phases


def save_plot(times, temps, volts, phases, out_path):
    labels = _PHASE_LABELS_KO if KOREAN_FONT_AVAILABLE else _PHASE_LABELS_EN

    fig, ax1 = plt.subplots(figsize=(11, 6))

    seen_labels = set()
    start_idx = 0
    for i in range(1, len(phases) + 1):
        if i == len(phases) or phases[i] != phases[start_idx]:
            phase = phases[start_idx]
            seg_times = times[start_idx:i]
            seg_temps = temps[start_idx:i]
            label = labels.get(phase, f"Phase{phase}") if phase not in seen_labels else None
            color = _PHASE_COLORS.get(phase, "#666666")
            ax1.plot(seg_times, seg_temps, color=color, linewidth=1.5, label=label)
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


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_CSV_FILE
    if len(sys.argv) <= 1:
        print(f"[안내] 인자가 없어 기본 파일을 사용합니다: {path}")

    times, temps, volts, phases = load_csv(path)
    if not times:
        print(f"[오류] {path}에서 읽은 데이터가 없습니다. 파일 경로를 확인하세요.")
        return

    out_path = path.rsplit(".", 1)[0] + "_replot.png"
    save_plot(times, temps, volts, phases, out_path)
    print(f"그래프 저장 완료: {out_path}")


if __name__ == "__main__":
    main()
