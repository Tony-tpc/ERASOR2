#!/usr/bin/env bash
# Run one ERASOR2 sequence from preprocessing through static-map generation.
#
# Required inputs:
#   1. A sequence directory containing velodyne/ and a pose file.
#   2. An ERASOR2 YAML parameter file.
#
# Example:
#   ./scripts/run_sequence.sh \
#     /data/erasor2_dataset/dataset/sequences/00 \
#     config/erasor2/seq_00.yaml

set -Eeuo pipefail

usage() {
  cat <<'EOF'
Usage:
  scripts/run_sequence.sh <sequence_dir> <parameter.yaml>

The sequence directory must contain:
  velodyne/000000.bin ...
  poses_suma_optim.txt       (SemanticKITTI/custom data)
  poses.txt                  (when dataset_name is HeLiPR)

The script automatically:
  1. checks and, when necessary, builds the C++ binaries;
  2. creates missing dummy labels without overwriting real labels;
  3. generates Patchwork++ ground labels and HDBSCAN instance labels;
  4. runs mapgen and run_erasor2;
  5. writes all results and the effective YAML under erasor2_output/.

Optional environment variables:
  ERASOR2_OUTPUT_DIR   output directory (default: <sequence_dir>/erasor2_output)
  ERASOR2_BUILD_DIR    CMake build directory (default: <repo>/build)
  ERASOR2_PYTHON       Python with PyYAML/Open3D/Patchwork++/HDBSCAN installed
  ERASOR2_CONDA_ENV    path to the Python environment (alternative to above)
  ERASOR2_JOBS         parallel build jobs
  ERASOR2_FORCE_LABELS=1  regenerate Patchwork/HDBSCAN labels
EOF
}

if [[ ${1:-} == "-h" || ${1:-} == "--help" ]]; then
  usage
  exit 0
fi
if [[ $# -ne 2 ]]; then
  usage >&2
  exit 2
fi

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "$SCRIPT_DIR/.." && pwd)"

resolve_path() {
  local input_path=$1
  local parent
  local leaf
  parent="$(cd -- "$(dirname -- "$input_path")" && pwd)"
  leaf="$(basename -- "$input_path")"
  printf '%s/%s\n' "$parent" "$leaf"
}

SEQUENCE_DIR="$(resolve_path "$1")"
PARAM_FILE="$(resolve_path "$2")"
OUTPUT_DIR="${ERASOR2_OUTPUT_DIR:-$SEQUENCE_DIR/erasor2_output}"
BUILD_DIR="${ERASOR2_BUILD_DIR:-$REPO_ROOT/build}"

if [[ ! -d "$SEQUENCE_DIR/velodyne" ]]; then
  echo "错误：sequence 下找不到 velodyne/：$SEQUENCE_DIR" >&2
  exit 2
fi
if [[ ! -f "$PARAM_FILE" ]]; then
  echo "错误：参数文件不存在：$PARAM_FILE" >&2
  exit 2
fi

if [[ -n ${ERASOR2_PYTHON:-} ]]; then
  PYTHON="$ERASOR2_PYTHON"
elif [[ -n ${ERASOR2_CONDA_ENV:-} && -x ${ERASOR2_CONDA_ENV}/bin/python ]]; then
  PYTHON="${ERASOR2_CONDA_ENV}/bin/python"
elif [[ -n ${VIRTUAL_ENV:-}${CONDA_PREFIX:-} ]] && command -v python >/dev/null 2>&1; then
  PYTHON="$(command -v python)"
elif [[ -x "$REPO_ROOT/.venv/bin/python" ]]; then
  PYTHON="$REPO_ROOT/.venv/bin/python"
else
  PYTHON="$(command -v python3 || true)"
fi
if [[ -z "$PYTHON" || ! -x "$PYTHON" ]]; then
  echo "错误：找不到 Python。可用 ERASOR2_PYTHON 指定解释器。" >&2
  exit 2
fi
if ! "$PYTHON" -c 'import yaml' >/dev/null 2>&1; then
  echo "错误：$PYTHON 缺少 PyYAML（安装命令：python -m pip install PyYAML）。" >&2
  exit 2
fi

mkdir -p "$OUTPUT_DIR"
EFFECTIVE_CONFIG="$OUTPUT_DIR/effective_config.yaml"
METADATA_FILE="$OUTPUT_DIR/.sequence_run_metadata"

echo "[1/6] 检查数据并生成生效配置"
"$PYTHON" - "$SEQUENCE_DIR" "$PARAM_FILE" "$OUTPUT_DIR" \
  "$EFFECTIVE_CONFIG" "$METADATA_FILE" <<'PY'
import sys
import shlex
from pathlib import Path

import yaml

sequence_dir = Path(sys.argv[1]).resolve()
parameter_file = Path(sys.argv[2]).resolve()
output_dir = Path(sys.argv[3]).resolve()
effective_config = Path(sys.argv[4])
metadata_file = Path(sys.argv[5])

with parameter_file.open("r", encoding="utf-8") as stream:
    config = yaml.safe_load(stream) or {}
if not isinstance(config, dict):
    raise SystemExit("参数文件顶层必须是 YAML mapping")

dataloader = config.setdefault("dataloader", {})
if not isinstance(dataloader, dict):
    raise SystemExit("参数文件中的 dataloader 必须是 YAML mapping")

dataset_name = str(dataloader.get("dataset_name", "SemanticKITTI"))
sequence = sequence_dir.name
interval = int(dataloader.get("accum_interval", 2))
start = int(config.get("start_frame", 0))
requested_end = int(config.get("end_frame", -1))
if interval < 1:
    raise SystemExit("dataloader.accum_interval 必须 >= 1")
if start < 0:
    raise SystemExit("start_frame 必须 >= 0")

scan_by_frame = {}
for path in (sequence_dir / "velodyne").glob("*.bin"):
    try:
        frame = int(path.stem)
    except ValueError:
        continue
    scan_by_frame[frame] = path
if not scan_by_frame:
    raise SystemExit("velodyne/ 中没有找到数字帧名的 .bin 文件")

if dataset_name == "HeLiPR":
    pose_path = sequence_dir / "poses.txt"
elif sequence == "19":
    pose_path = sequence_dir / "kiss_icp_poses.txt"
else:
    pose_path = sequence_dir / "poses_suma_optim.txt"
if not pose_path.is_file():
    raise SystemExit("找不到位姿文件：{}".format(pose_path))

with pose_path.open("r", encoding="utf-8") as stream:
    pose_count = sum(1 for line in stream if line.strip())

# C++ reads poses through end_frame + accum_interval - 1. Aligning the end
# frame to the accumulation interval also prevents it from requesting one
# extra scan beyond an unpadded custom sequence.
max_scan = max(scan_by_frame)
max_safe_end = min(max_scan, pose_count - interval)
candidate_end = max_safe_end if requested_end < 0 else min(requested_end, max_safe_end)
end = start + ((candidate_end - start) // interval) * interval
if end < start:
    raise SystemExit(
        "可用数据不足：start_frame={}，最大安全 end_frame={}".format(
            start, max_safe_end
        )
    )

missing_scans = [
    frame for frame in range(start, end + 1) if frame not in scan_by_frame
]
if missing_scans:
    preview = ", ".join("{:06d}".format(x) for x in missing_scans[:8])
    raise SystemExit("处理区间存在缺失点云帧：{}".format(preview))

for frame in range(start, end + 1):
    size = scan_by_frame[frame].stat().st_size
    if size == 0 or size % 16 != 0:
        raise SystemExit(
            "点云必须是非空 float32 x/y/z/intensity：{} ({} bytes)".format(
                scan_by_frame[frame], size
            )
        )

if requested_end != end:
    print(
        "提示：为匹配现有点云、位姿和 accum_interval，end_frame 从 {} 调整为 {}".format(
            requested_end, end
        ),
        file=sys.stderr,
    )

dataloader["abs_data_dir"] = str(sequence_dir.parent)
dataloader["sequence"] = sequence
dataloader["abs_save_dir"] = str(output_dir)
dataloader.setdefault("instance_seg_method", "hdbscan")
config["start_frame"] = start
config["end_frame"] = end

# A missing rerun section currently enables and spawns the GUI through C++
# defaults. Unified runs should be unattended unless the parameter file asks
# for visualization explicitly.
rerun = config.setdefault("rerun", {})
if not isinstance(rerun, dict):
    raise SystemExit("参数文件中的 rerun 必须是 YAML mapping")
rerun.setdefault("enabled", False)
rerun.setdefault("spawn", False)

with effective_config.open("w", encoding="utf-8") as stream:
    yaml.safe_dump(config, stream, sort_keys=False, allow_unicode=True)
with metadata_file.open("w", encoding="utf-8") as stream:
    stream.write("START={}\n".format(shlex.quote(str(start))))
    stream.write("END={}\n".format(shlex.quote(str(end))))
    stream.write("INTERVAL={}\n".format(shlex.quote(str(interval))))
    stream.write("SEQUENCE={}\n".format(shlex.quote(sequence)))
    stream.write("DATASET_NAME={}\n".format(shlex.quote(dataset_name)))
    stream.write(
        "INSTANCE_METHOD={}\n".format(
            shlex.quote(str(dataloader["instance_seg_method"]))
        )
    )

print("  sequence : {}".format(sequence_dir))
print("  frames   : {}..{} (interval {})".format(start, end, interval))
print("  poses    : {} ({} lines)".format(pose_path, pose_count))
print("  config   : {}".format(effective_config))
print("  output   : {}".format(output_dir))
PY

# The metadata file contains shell-safe numeric/simple values generated above.
# shellcheck disable=SC1090
source "$METADATA_FILE"

echo "[2/6] 检查 C++ 程序"
if [[ ! -x "$BUILD_DIR/mapgen" || ! -x "$BUILD_DIR/run_erasor2" ]]; then
  echo "  未找到完整 build，开始编译……"
  cmake -S "$REPO_ROOT" -B "$BUILD_DIR"
  cmake --build "$BUILD_DIR" --parallel "${ERASOR2_JOBS:-$(nproc)}"
else
  echo "  使用已有 build：$BUILD_DIR"
fi

echo "[3/6] 补齐 mapgen 所需标签"
"$PYTHON" - "$SEQUENCE_DIR" "$START" "$END" <<'PY'
import sys
from pathlib import Path

sequence_dir = Path(sys.argv[1])
start = int(sys.argv[2])
end = int(sys.argv[3])
label_dir = sequence_dir / "labels"
label_dir.mkdir(parents=True, exist_ok=True)
created = 0
for frame in range(start, end + 1):
    scan = sequence_dir / "velodyne" / "{:06d}.bin".format(frame)
    label = label_dir / "{:06d}.label".format(frame)
    point_count = scan.stat().st_size // 16
    expected_size = point_count * 4
    if label.exists():
        if label.stat().st_size != expected_size:
            raise SystemExit(
                "标签点数与点云不一致：{} (expected {} bytes, got {})".format(
                    label, expected_size, label.stat().st_size
                )
            )
        continue
    # A sparse zero-filled file is a valid uint32 dummy SemanticKITTI label.
    # Existing real labels are deliberately never overwritten.
    with label.open("wb") as stream:
        stream.truncate(expected_size)
    created += 1
print("  新建 {} 个 dummy label；已有 label 保持不变".format(created))
PY

labels_complete() {
  "$PYTHON" - "$SEQUENCE_DIR" "$START" "$END" "$INSTANCE_METHOD" <<'PY'
import sys
from pathlib import Path

root = Path(sys.argv[1])
start, end = int(sys.argv[2]), int(sys.argv[3])
instance_method = sys.argv[4]
for frame in range(start, end + 1):
    scan_size = (root / "velodyne" / "{:06d}.bin".format(frame)).stat().st_size
    expected = scan_size // 4
    for folder in ("patchwork", instance_method):
        label = root / folder / "{:06d}.label".format(frame)
        if not label.is_file() or label.stat().st_size != expected:
            raise SystemExit(1)
PY
}

echo "[4/6] 生成 Patchwork++ / HDBSCAN 标签"
if [[ ${ERASOR2_FORCE_LABELS:-0} != 1 ]] && labels_complete; then
  echo "  标签完整，跳过预处理"
else
  if [[ "$INSTANCE_METHOD" != "hdbscan" ]]; then
    echo "错误：自动预处理仅生成 hdbscan，当前参数为 $INSTANCE_METHOD。" >&2
    echo "请提供完整的 patchwork/$INSTANCE_METHOD 标签，或改用 hdbscan。" >&2
    exit 2
  fi
  if ! "$PYTHON" -c 'import hdbscan, matplotlib, numpy, open3d, pypatchworkpp, sklearn, tqdm' \
      >/dev/null 2>&1; then
    echo "错误：预处理 Python 环境缺少 Open3D/Patchwork++/HDBSCAN 等依赖。" >&2
    echo "请激活 erasor2 环境，或用 ERASOR2_PYTHON 指定它的 Python。" >&2
    exit 2
  fi
  "$PYTHON" "$SCRIPT_DIR/kitti_clustering.py" \
    --sequence-dir "$SEQUENCE_DIR" \
    --seq "$SEQUENCE" \
    --init_stamp "$START" \
    --end_stamp "$END" \
    --save-instance-labels \
    --save-ground-labels
  labels_complete
fi

echo "[5/6] 生成原始累计地图（mapgen）"
"$BUILD_DIR/mapgen" "$EFFECTIVE_CONFIG"

echo "[6/6] 去除动态物体并生成静态地图（ERASOR2）"
"$BUILD_DIR/run_erasor2" "$EFFECTIVE_CONFIG"

echo
echo "完成。输出目录：$OUTPUT_DIR"
find "$OUTPUT_DIR" -maxdepth 1 -type f \( -name '*.pcd' -o -name '*.yaml' \) \
  -printf '  %f\n' | sort
