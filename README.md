# ERASOR2：动态物体去除与静态点云地图生成

本仓库在 [url-kaist/ERASOR2](https://github.com/url-kaist/ERASOR2) 的算法基础上，
提供了一套面向离线 LiDAR sequence 的完整处理流程。输入逐帧点云、LiDAR 位姿和
一份 YAML 参数文件，即可生成原始累计地图、动态物体过滤后的静态地图，以及逐帧
MOS 标签。

```text
逐帧点云 + LiDAR 位姿
        │
        ├── Patchwork++：地面分割
        ├── HDBSCAN：实例聚类
        │
        ├── mapgen：生成原始累计地图
        │
        └── ERASOR2：检测并去除动态物体
                    ├── 静态点云地图
                    └── 逐帧 MOS 标签
```

## 相比 url-kaist 原版的主要改进

本分支的优势主要是工程化、数据接入和长序列运行能力。ERASOR2 的核心思想仍来自
原论文；下列改进不代表在所有数据集上必然获得更高的 PR、RR 或 F1。

| 项目 | url-kaist 原版 | 当前版本 |
|---|---|---|
| 构建与运行 | 依赖 ROS1、catkin、rosparam 和 roslaunch | 纯 CMake/C++17，程序直接读取 YAML，不需要 ROS master 或 catkin 工作空间 |
| Grid Map | 依赖 ROS 生态中的 `grid_map` 包 | 内置算法实际需要的轻量 Grid Map 实现，减少系统依赖 |
| 自采 sequence 接入 | 通常需要手工整理路径、标签并逐项启动 | `run_sequence.sh` 只接收 sequence 路径和参数文件，自动完成检查、预处理与执行 |
| 位姿输入 | SemanticKITTI/SuMa 相机位姿转换流程 | 直接读取 `T_map_lidar`，支持 KITTI 3×4 和时间戳加四元数两种格式，更适合直接输出 LiDAR 位姿的系统 |
| 长序列内存 | 主要以内存中的完整序列和地图运行 | 提供三遍式 external-memory streaming，按阈值压缩全局地图，并仅保留时间窗口 |
| 可视化 | RViz、TF 和 ROS publisher | Rerun 可选；默认可完全关闭，适合无桌面的服务器和批处理 |
| 输出与复现 | 主要面向论文数据集流程 | 同时输出静态地图和逐帧 MOS 标签；提供 pipeline、benchmark 和 parity 检查工具 |
| ERASOR 版本 | ERASOR2 | 同一次 CMake 构建还可生成 ERASOR v1 的 `run_erasor`，便于横向比较 |

当前版本把位姿文件解释为直接的 `T_map_lidar`。如果已有的是相机位姿或其他传感器
坐标系位姿，必须先用外参转换到 LiDAR 位姿；否则轨迹方向和点云地图可能不一致。

## 运行环境

推荐 Ubuntu 20.04 或 22.04。C++ 部分需要：

```bash
sudo apt-get update
sudo apt-get install -y \
  build-essential cmake git \
  libpcl-dev libeigen3-dev libopencv-dev libomp-dev \
  libboost-system-dev libboost-filesystem-dev \
  libyaml-cpp-dev libopenmpi-dev
```

Python 部分用于 Patchwork++、HDBSCAN 预处理和结果评估。推荐使用仓库提供的 Conda
环境：

```bash
conda env create -f scripts/environment.yml
conda activate erasor2
```

也可以使用 venv：

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -U pip setuptools wheel
python -m pip install -r scripts/requirements.txt PyYAML
```

### 编译

默认构建为无可视化的 headless 版本，不会下载 Rerun C++ SDK：

```bash
cmake -S . -B build
cmake --build build -j"$(nproc)"
```

主要生成以下程序：

| 程序 | 用途 |
|---|---|
| `build/mapgen` | 累积原始扫描并生成原始/体素化地图 |
| `build/run_erasor2` | 运行 ERASOR2，输出静态地图和 MOS 标签 |
| `build/run_erasor` | 运行 ERASOR v1，便于与 ERASOR2 对比 |
| `build/compare_map` | 比较地图 |
| `build/accum_4dmos` | 累积外部 4D-MOS 结果 |

如需 Rerun 可视化，配置时显式开启：

```bash
cmake -S . -B build -DERASOR2_ENABLE_RERUN=ON
cmake --build build -j"$(nproc)"
```

## 输入数据

### Sequence 目录结构

推荐使用 SemanticKITTI 风格目录。一个 sequence 应类似：

```text
/data/sequences/00/
├── velodyne/
│   ├── 000000.bin
│   ├── 000001.bin
│   └── ...
├── poses_suma_optim.txt
├── times.txt                  # 可选，当前核心程序不读取
├── labels/                    # 真实语义标签或 dummy label
├── patchwork/                 # 可由脚本生成
└── hdbscan/                   # 可由脚本生成
```

帧文件建议从 `000000` 开始连续编号。配置中的 frame 编号直接用于寻找
`%06d.bin` 和 `%06d.label`。

### 输入文件格式

| 输入 | 格式 | 是否必需 |
|---|---|---|
| `velodyne/NNNNNN.bin` | 连续 `float32`，每点为 `x y z intensity`，共 16 字节 | 必需 |
| `poses_suma_optim.txt` | 每行一个直接的 `T_map_lidar` | SemanticKITTI/custom 模式必需 |
| `poses.txt` | 每行一个直接的 LiDAR 位姿 | `dataset_name: HeLiPR` 时必需 |
| `labels/NNNNNN.label` | 每点一个 `uint32` SemanticKITTI 标签 | `mapgen` 必需；无真值时可使用全零 dummy label |
| `patchwork/NNNNNN.label` | 每点一个 `uint32`，非零表示地面 | `run_erasor2` 必需，可自动生成 |
| `hdbscan/NNNNNN.label` | 每点一个 `uint32`，高 16 位保存 instance ID | 使用 HDBSCAN 时必需，可自动生成 |

位姿支持两种行格式：

```text
# KITTI row-major 3x4，共 12 个数
r00 r01 r02 tx r10 r11 r12 ty r20 r21 r22 tz

# 时间戳 + 平移 + 四元数，共 8 个数
timestamp x y z qx qy qz qw
```

点云与位姿必须使用同一个 LiDAR 坐标系、相同帧顺序和相同时间基准。代码不会进行
点云去畸变、时间同步或外参标定。

### Dummy label 的含义

自采数据没有语义真值时，可以为每个点写入 `uint32(0)`。这样可以让 `mapgen`
生成几何累计地图，但该地图不包含真实的动态/静态语义，因此不能据此计算可信的
Preservation、Rejection 或 F1。已有真实 `labels/` 时不要替换为 dummy label。

## 输出数据

假设 sequence 为 `00`，处理区间为 `0..3450`，输出目录包含：

```text
erasor2_output/
├── effective_config.yaml                         # 仅一键脚本生成
├── 00_0_to_3450_w_interval_1_voxel_0_1_original.pcd
├── 00_0_to_3450_w_interval_1_voxel_0_1.pcd
├── 00_0_frame_0_to_3450_streaming_estimated.pcd # streaming 模式
└── mos/
    ├── 000000.label
    ├── 000001.label
    └── ...
```

| 输出 | 含义 |
|---|---|
| `*_original.pcd` | `mapgen` 生成的未做最终 map voxel 的累计地图 |
| `*_w_interval_*_voxel_*.pcd` | `mapgen` 生成的体素化原始累计地图 |
| `*_streaming_estimated.pcd` | streaming 模式生成的静态地图 |
| `*_estimated.pcd` | 关闭 streaming 时生成的静态地图 |
| `mos/*.label` | SemanticKITTI MOS 风格标签：静态为 `0`，动态为 `251` |
| `effective_config.yaml` | 一键脚本覆盖数据路径、sequence、输出路径和安全帧范围后的实际配置 |

## 一键运行：推荐方式

只需要准备 sequence 路径和参数文件：

```bash
./scripts/run_sequence.sh \
  /data/sequences/00 \
  config/erasor2/seq_00.yaml
```

脚本会依次完成：

1. 检查点云文件、位姿数量、帧区间和点云字节数。
2. 根据需要执行 CMake 编译。
3. 为缺少语义真值的帧创建 dummy label，不覆盖已有 label。
4. 检查并复用已有 Patchwork/HDBSCAN 标签；缺失时自动生成。
5. 运行 `mapgen`。
6. 运行 `run_erasor2`。

脚本不会修改传入的 YAML，而是把路径相关字段写入输出目录中的
`effective_config.yaml`：

- `dataloader.abs_data_dir` 自动设为 sequence 的父目录；
- `dataloader.sequence` 自动设为 sequence 目录名；
- `dataloader.abs_save_dir` 自动设为输出目录；
- `end_frame: -1` 时自动选择当前数据能安全处理到的最后一帧；
- 指定的末帧超出点云或位姿范围时，自动向下收缩并按 `accum_interval` 对齐。

默认输出到 `<sequence>/erasor2_output/`。常用环境变量：

```bash
ERASOR2_OUTPUT_DIR=/data/output \
ERASOR2_PYTHON=/opt/conda/envs/erasor2/bin/python \
ERASOR2_BUILD_DIR=./build \
./scripts/run_sequence.sh /data/sequences/00 config/erasor2/seq_00.yaml
```

| 环境变量 | 作用 |
|---|---|
| `ERASOR2_OUTPUT_DIR` | 修改输出目录 |
| `ERASOR2_PYTHON` | 指定带预处理依赖的 Python |
| `ERASOR2_CONDA_ENV` | 指定 Conda 环境根目录，作为 `ERASOR2_PYTHON` 的替代 |
| `ERASOR2_BUILD_DIR` | 指定 CMake build 目录 |
| `ERASOR2_JOBS` | 指定并行编译任务数 |
| `ERASOR2_FORCE_LABELS=1` | 强制重新生成 Patchwork/HDBSCAN 标签 |

如果当前参数使用 `instance_seg_method: cais`，一键脚本只会复用已有 CAIS 标签，
不会自动生成 CAIS；自动预处理目前只生成 HDBSCAN。

## 手动分步运行

手动运行适合调试预处理结果、单独调参或复用已有标签。以下示例假设：

```bash
SEQ=/data/sequences/00
CFG=config/erasor2/seq_00.yaml
START=0
END=3450
```

### 1. 配置 YAML

手动运行时必须在 YAML 中填写真实值；与一键脚本不同，C++ 程序不会自动解析
`end_frame: -1`。

```yaml
start_frame: 0
end_frame: 3450

dataloader:
  dataset_name: "SemanticKITTI"
  abs_data_dir: "/data/sequences"  # 直接包含 00/、01/ 等 sequence
  sequence: "00"
  abs_save_dir: "/data/output"
  instance_seg_method: "hdbscan"
  accum_interval: 1
  voxel_size: 0.1
  map_voxel_size: 0.1
  expansion_range: 0

streaming:
  enabled: true
  compact_threshold_points: 3000000
  use_gt_labels: false

rerun:
  enabled: false
  spawn: false
  save_path: ""
```

当 `accum_interval > 1` 时，位姿文件至少要覆盖到
`end_frame + accum_interval - 1`。处理裁剪后的 sequence 时建议把
`dataloader.expansion_range` 设为 `0`，避免读取区间外帧。

### 2. 生成 dummy label（仅无语义真值时）

下面的命令仅创建缺失文件，并检查已有 label 的点数，不覆盖真实标签：

```bash
python3 - "$SEQ" "$START" "$END" <<'PY'
import sys
from pathlib import Path

seq = Path(sys.argv[1])
start, end = int(sys.argv[2]), int(sys.argv[3])
labels = seq / "labels"
labels.mkdir(parents=True, exist_ok=True)

for frame in range(start, end + 1):
    scan = seq / "velodyne" / f"{frame:06d}.bin"
    label = labels / f"{frame:06d}.label"
    expected = scan.stat().st_size // 4
    if label.exists():
        if label.stat().st_size != expected:
            raise RuntimeError(f"label 点数不一致: {label}")
        continue
    with label.open("wb") as stream:
        stream.truncate(expected)
PY
```

计算依据是每个点在 `.bin` 中占 16 字节，在 `.label` 中占 4 字节，因此 label
文件大小应等于 bin 文件大小的四分之一。

### 3. 生成地面和实例标签

```bash
python scripts/kitti_clustering.py \
  --sequence-dir "$SEQ" \
  --seq "$(basename "$SEQ")" \
  --init_stamp "$START" \
  --end_stamp "$END" \
  --save-ground-labels \
  --save-instance-labels
```

输出写入：

```text
$SEQ/patchwork/NNNNNN.label
$SEQ/hdbscan/NNNNNN.label
```

HDBSCAN 聚类质量会直接影响动态物体去除效果。正式运行前，建议抽查若干帧：

```bash
python scripts/visualize_clustering.py \
  --kitti_dir /data \
  --seq 00 \
  --init_stamp "$START" \
  --end_stamp "$END"
```

上述 `--kitti_dir` 遵循 SemanticKITTI 根目录约定，即点云应位于
`<kitti_dir>/dataset/sequences/<seq>/velodyne/`。如果数据不采用该布局，可以把
sequence 临时整理到标准目录后再使用可视化脚本。

### 4. 检查输入数量

```bash
find "$SEQ/velodyne" -maxdepth 1 -name '*.bin' | wc -l
wc -l < "$SEQ/poses_suma_optim.txt"
find "$SEQ/labels"    -maxdepth 1 -name '*.label' | wc -l
find "$SEQ/patchwork" -maxdepth 1 -name '*.label' | wc -l
find "$SEQ/hdbscan"   -maxdepth 1 -name '*.label' | wc -l
```

除文件数量外，每一帧的三个 label 文件都必须与对应点云包含相同点数。

### 5. 生成地图

```bash
mkdir -p /data/output

./build/mapgen "$CFG"
./build/run_erasor2 "$CFG"
```

如果只修改了 ERASOR2 检测阈值且原始累计地图已经存在，可以只重新运行
`run_erasor2`。如果修改了 frame 范围、体素大小、外参或输入数据，应重新运行
`mapgen`。

### 6. 评估（仅有真实语义标签时）

```bash
python scripts/evaluate.py \
  --gt  /data/output/00_0_to_3450_w_interval_1_voxel_0_1.pcd \
  --est /data/output/00_0_frame_0_to_3450_streaming_estimated.pcd \
  --vox 0.1
```

`evaluate.py` 输出 Preservation、Rejection 和 F1。使用 dummy label 得到的指标没有
有效语义，不应作为算法性能结论。

## 关键参数

参数示例位于 `config/erasor2/`。常用字段如下：

| 参数 | 说明 |
|---|---|
| `start_frame` / `end_frame` | 处理帧范围；`-1` 自动末帧只由一键脚本支持 |
| `dataloader.accum_interval` | 帧采样间隔 |
| `dataloader.voxel_size` | `mapgen` 累积过程使用的体素大小 |
| `dataloader.map_voxel_size` | 最终输出地图的体素大小 |
| `dataloader.expansion_range` | 轨迹分段后的扩展帧范围；裁剪数据通常设为 `0` |
| `extrinsic.rotation` / `translation` | 输入 LiDAR 到算法使用坐标系的外参 |
| `extrinsic.robot_body_size` | 剔除车体附近点的范围 |
| `erasor2.range_of_interest` | 动态检测关注范围 |
| `erasor2.min_z_voi` / `max_z_voi` | 输入点云的高度范围 |
| `erasor2.scan_ratio_threshold` | Scan Ratio 判定阈值；越大通常越激进 |
| `erasor2.log_odds.*` | 占据更新参数 |
| `erasor2.region_proposal_thr` | 动态候选区域阈值 |
| `erasor2.moving_object_detection.*` | 实例动态得分阈值 |
| `erasor2.volumetric_outlier_removal.*` | 体积离群点过滤与时间窗口参数 |
| `streaming.enabled` | 开启长序列三遍式 streaming |
| `streaming.compact_threshold_points` | 全局点数达到该值时执行保标签体素压缩 |
| `rerun.enabled` | 是否记录 Rerun 可视化数据 |

阈值需要根据传感器线数、点密度、安装高度、场景尺度和实例聚类质量调整。仓库中的
配置是示例，不应直接视为所有设备的最佳参数。

## Streaming 与普通模式

推荐长 sequence 使用：

```yaml
streaming:
  enabled: true
  compact_threshold_points: 3000000
  use_gt_labels: false
```

Streaming 执行三个 pass：

1. 重放扫描并构建可压缩的全局实例地图。
2. 再次重放扫描，更新全局地面/可通行栅格。
3. 只加载局部时间窗口执行动态检测，逐帧写出 MOS，并累积静态地图。

该模式通过定期体素压缩和窗口化处理降低长序列峰值内存，但通常会增加磁盘读取与
运行时间。`streaming.enabled: false` 会使用普通内存模式，并生成不带
`_streaming` 后缀的静态地图。

## 可视化

只有使用 `-DERASOR2_ENABLE_RERUN=ON` 编译时，C++ Rerun 输出才会生效。示例：

```yaml
rerun:
  enabled: true
  spawn: true
  save_path: ""
```

无桌面环境可以写入 `.rrd`：

```yaml
rerun:
  enabled: true
  spawn: false
  save_path: "/data/output/erasor2.rrd"
```

批处理时建议保持 `enabled: false`。

## 常见问题

### 找不到 `hdbscan/*.label` 或 `patchwork/*.label`

预处理没有完成，或 label 点数与点云不一致。重新运行
`scripts/kitti_clustering.py`，也可以用一键脚本自动检查与生成。

### 点云与 label 数量不一致

`.bin` 每点是 4 个 `float32`，`.label` 每点是 1 个 `uint32`，所以：

```text
label 文件字节数 = bin 文件字节数 / 4
```

不要把其他点格式直接改扩展名为 `.bin`。

### 地图方向错误或墙体重影

依次检查：

1. 位姿是否为直接的 `T_map_lidar`，而不是 `T_map_camera` 或 `T_lidar_map`。
2. 点云与位姿是否严格对应同一帧、同一时间。
3. 点云是否已正确去畸变。
4. 多 LiDAR 数据是否已完成时间同步和外参变换。
5. `extrinsic` 是否与当前点云坐标系一致。

### 手动运行时 `end_frame: -1` 失败

自动末帧属于 `run_sequence.sh` 的功能。直接调用 C++ 程序时必须在 YAML 中填写
明确的非负 `end_frame`。

### Python 找不到 Open3D、Patchwork++ 或 HDBSCAN

先激活正确环境，或者给一键脚本指定解释器：

```bash
ERASOR2_PYTHON=/opt/conda/envs/erasor2/bin/python \
  ./scripts/run_sequence.sh /data/sequences/00 config/erasor2/seq_00.yaml
```

### 内存不足

开启 `streaming.enabled`，适当降低 `compact_threshold_points`，增大
`map_voxel_size`，或缩小处理帧范围。阈值过低会增加反复体素化的计算成本。

## 其他运行入口

已经准备好 YAML 中的所有路径和标签时，可以使用 Python pipeline 串联
`mapgen → run_erasor2 → evaluate`：

```bash
python scripts/run_pipeline.py \
  --config config/erasor2/seq_05.yaml \
  --conda-env /opt/conda/envs/erasor2
```

当前 `run_pipeline.py` 按普通模式的 `*_estimated.pcd` 文件名执行评估，因此使用
该入口时请在对应 YAML 中设置 `streaming.enabled: false`。一键脚本不受此限制。

多 sequence 或 ERASOR v1/v2 对比可使用：

```bash
python scripts/run_benchmark.py --algorithm both --build-dir ./build
```

这些入口面向带真实语义标签的评估数据；一般自采 sequence 优先使用
`scripts/run_sequence.sh`。

## 仓库结构

```text
config/       YAML 参数文件
include/      C++ 头文件
src/          mapgen、ERASOR/ERASOR2 和数据加载源码
scripts/      预处理、评估、一键执行和 benchmark 脚本
tests/        parity/regression 检查
docker/       容器辅助文件
launch/       历史 launch 文件；当前 CMake 构建不使用
rviz/         历史 RViz 配置；当前推荐使用 Rerun
```

## 致谢与引用

本仓库基于 [url-kaist/ERASOR2](https://github.com/url-kaist/ERASOR2)，并使用
PCL、Eigen、OpenCV、Patchwork++、HDBSCAN、nanoflann 和 Rerun 等开源项目。

如在学术工作中使用，请引用 ERASOR2 和 ERASOR：

```bibtex
@article{lim2025erasor2,
  title   = {{ERASOR2}: Instance-Aware Robust 3D Mapping of the Static World in Dynamic Scenes},
  author  = {Lim, Hyungtae and others},
  journal = {IEEE Robotics and Automation Letters},
  year    = {2025}
}

@article{lim2021erasor,
  title   = {{ERASOR}: Egocentric Ratio of Pseudo Occupancy-based Dynamic Object Removal for Static 3D Point Cloud Map Building},
  author  = {Lim, Hyungtae and Hwang, Sungwon and Myung, Hyun},
  journal = {IEEE Robotics and Automation Letters},
  volume  = {6},
  number  = {2},
  pages   = {2272--2279},
  year    = {2021}
}
```

许可证见 [Licence](Licence)。
