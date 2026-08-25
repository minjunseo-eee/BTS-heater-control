"""
1단계-b 정전류 테스트 결과 비교/해석 스크립트
- step1b_current_step_test.py로 여러 전류(0.5A, 1A, 2A, 3A ...)를 각각 세션별로 테스트한 뒤,
  그 CSV 파일들을 한 번에 모아서 비교 분석함

사용 방법:
    python3 analyze_step1b_results.py step1b_0.5A_*.csv step1b_1A_*.csv step1b_2A_*.csv step1b_3A_*.csv

만들어지는 것:
1) 콘솔에 전류별 요약표 (85도 도달까지 걸린 시간, CC모드 유지 비율, 정상상태 평균온도 등)
2) 온도-시간 비교 그래프 (모든 전류를 한 그래프에 겹쳐서, 어느 전류가 더 빨리/높이 올라가는지 비교)
"""

import sys
import csv
import glob
from collections import defaultdict

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager

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

TEMP_CUTOFF = 85.0
STEADY_WINDOW_S = 90.0

# --- 커맨드라인 인자 없이 실행할 때(예: VS Code 디버거) 쓰일 기본 파일 목록 ---
# 실제 생성된 CSV 파일명으로 채워둠. 새로 테스트를 추가하면 여기에 파일명만 추가하면 됨.
DEFAULT_CSV_FILES = [
    "step1b_0.5A_20260819_151838.csv",
    "step1b_0.6A_20260819_154338.csv",
    "step1b_0.7A_20260819_153638.csv",
    "step1b_1A_20260819_153145.csv",
]


def load_csv(path):
    rows = []
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                rows.append({
                    "elapsed_s": float(row["elapsed_s"]),
                    "target_current": float(row["target_current"]),
                    "voltage_meas": float(row["voltage_meas"]),
                    "current_meas": float(row["current_meas"]),
                    "power_meas": float(row["power_meas"]),
                    "cv_cc_status": row["cv_cc_status"],
                    "temperature_c": float(row["temperature_c"]),
                    "resistance_ohm": float(row["resistance_ohm"]),
                })
            except (ValueError, KeyError):
                continue
    return rows


def summarize(rows):
    if not rows:
        return None

    target_current = rows[0]["target_current"]
    times = [r["elapsed_s"] for r in rows]
    temps = [r["temperature_c"] for r in rows]

    # 85도 도달 시간
    time_to_cutoff = None
    for r in rows:
        if r["temperature_c"] >= TEMP_CUTOFF:
            time_to_cutoff = r["elapsed_s"]
            break

    # CC 모드 유지 비율 (문자열에 CC가 포함된 비율)
    cc_count = sum(1 for r in rows if "CC" in r["cv_cc_status"])
    cc_ratio = cc_count / len(rows) if rows else 0.0

    # 실측 전류가 목표 전류에 얼마나 근접했는지 (평균 오차)
    current_errors = [abs(r["current_meas"] - target_current) for r in rows]
    mean_current_error = sum(current_errors) / len(current_errors)

    # 정상상태(마지막 STEADY_WINDOW_S초) 평균 온도
    end_time = times[-1]
    window_temps = [t for tm, t in zip(times, temps) if end_time - tm <= STEADY_WINDOW_S]
    steady_avg = sum(window_temps) / len(window_temps) if window_temps else None

    return {
        "target_current": target_current,
        "n_samples": len(rows),
        "duration_s": times[-1],
        "time_to_cutoff": time_to_cutoff,
        "cc_ratio": cc_ratio,
        "mean_current_error": mean_current_error,
        "steady_avg_temp": steady_avg,
        "final_temp": temps[-1],
    }


def resolve_paths(raw_args):
    """커맨드라인 인자를 받되, 없으면 DEFAULT_CSV_FILES를 쓰고, 각 인자는 와일드카드(*)도 허용."""
    raw_paths = raw_args if raw_args else DEFAULT_CSV_FILES
    if not raw_args:
        print("[안내] 커맨드라인 인자가 없어 스크립트 상단의 DEFAULT_CSV_FILES 목록을 사용합니다.")

    resolved = []
    for p in raw_paths:
        matches = glob.glob(p)
        if matches:
            resolved.extend(matches)
        else:
            resolved.append(p)  # 와일드카드가 아니거나 매칭 안 되면 그대로 시도 (아래에서 파일 없음 경고 처리)
    return resolved


def main():
    paths = resolve_paths(sys.argv[1:])
    if not paths:
        print("사용법: python3 analyze_step1b_results.py <csv1> <csv2> ...")
        return

    all_data = {}   # target_current -> rows
    summaries = []

    for path in paths:
        try:
            rows = load_csv(path)
        except FileNotFoundError:
            print(f"[경고] {path}: 파일을 찾을 수 없음, 건너뜀")
            continue
        if not rows:
            print(f"[경고] {path}: 읽을 데이터 없음, 건너뜀")
            continue
        summary = summarize(rows)
        summaries.append(summary)
        all_data[summary["target_current"]] = rows

    summaries.sort(key=lambda s: s["target_current"])

    # --- 콘솔 요약표 ---
    print("\n=== 전류별 요약 ===")
    header = f"{'전류(A)':>8} | {'85도 도달(s)':>12} | {'CC유지율':>8} | {'전류오차평균':>10} | {'정상상태평균온도':>16} | {'최종온도':>8}"
    print(header)
    print("-" * len(header))
    for s in summaries:
        cutoff_str = f"{s['time_to_cutoff']:.1f}" if s["time_to_cutoff"] is not None else "미도달"
        steady_str = f"{s['steady_avg_temp']:.2f}" if s["steady_avg_temp"] is not None else "N/A"
        print(
            f"{s['target_current']:>8.2f} | {cutoff_str:>12} | {s['cc_ratio']*100:>7.1f}% | "
            f"{s['mean_current_error']:>10.3f} | {steady_str:>16} | {s['final_temp']:>8.2f}"
        )

    # --- 비교 그래프 ---
    fig, ax = plt.subplots(figsize=(9, 5))
    colors = plt.cm.viridis(
        [i / max(len(all_data) - 1, 1) for i in range(len(all_data))]
    )
    for color, (current, rows) in zip(colors, sorted(all_data.items())):
        times = [r["elapsed_s"] for r in rows]
        temps = [r["temperature_c"] for r in rows]
        label = f"{current:g}A" if KOREAN_FONT_AVAILABLE else f"{current:g}A"
        ax.plot(times, temps, color=color, linewidth=1.3, label=label)

    ax.axhline(TEMP_CUTOFF, color="gray", linestyle=":", linewidth=1)

    if KOREAN_FONT_AVAILABLE:
        ax.set_xlabel("경과 시간 (s)")
        ax.set_ylabel("온도 (C)")
        ax.set_title("전류별 온도 응답 비교")
    else:
        ax.set_xlabel("Elapsed time (s)")
        ax.set_ylabel("Temperature (C)")
        ax.set_title("Temperature response by target current")

    ax.legend(loc="lower right")
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    out_path = "step1b_comparison.png"
    fig.savefig(out_path, dpi=150)
    plt.close(fig)

    print(f"\n비교 그래프 저장 완료: {out_path}")


if __name__ == "__main__":
    main()
