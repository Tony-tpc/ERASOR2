# Adaptive-LIO 导出 ERASOR2 数据集使用说明

本文档说明如何使用修改后的 **Adaptive-LIO_KITTI** 生成 ERASOR2 所需的数据集，并用修改后的 **ERASOR2** 完成动态障碍物去除，最终输出干净的静态点云地图。

---

## 1. 当前版本做了什么

当前 Adaptive-LIO 导出版的目标是：

```text
rosbag / 在线雷达数据
    ↓
Adaptive-LIO 建图/定位
    ↓
同步导出 ERASOR2 / KITTI 风格数据集
    ↓
ERASOR2 生成 ground label / instance label
    ↓
mapgen 构建初始地图
    ↓
run_erasor2 删除动态障碍物
    ↓
输出 clean / static map
```

当前版本保留了：

```text
1. 主雷达点云导出
2. aux 副雷达点云导出
3. cloud_pub.min_z_filter / max_z_filter 高度限制
4. 每帧 .bin 点云
5. 每帧 pose
6. times.txt
```

输出格式为 SemanticKITTI / KITTI 风格：

```text
erasor2_dataset/
└── dataset/
    └── sequences/
        └── 00/
            ├── velodyne/
            │   ├── 000000.bin
            │   ├── 000001.bin
            │   └── ...
            ├── poses_suma_optim.txt
            ├── times.txt
            ├── labels/
            ├── patchwork/
            └── hdbscan/
```

其中：

```text
velodyne/*.bin          每一帧点云，float32: x y z intensity
poses_suma_optim.txt    每一帧全局位姿，KITTI 3x4 row-major
times.txt               每一帧时间戳
labels/*.label          SemanticKITTI 语义标签，可用 dummy 全 0 标签
patchwork/*.label       Patchwork++ 生成的 ground label
hdbscan/*.label         HDBSCAN 生成的 instance label
```

---


## 3. Adaptive-LIO 侧配置

打开 Adaptive-LIO 配置，例如：

```bash
gedit config/mapping_m.yaml
```

确认 mapping 模块开启：

```yaml
mapping_module:
  enable_mapping: true
  export_erasor2: true
  erasor2_save_dir: "./erasor2_dataset/"
  sequence_id: "00"
  record_every_frame: true
```

确认 cloud_pub 高度限制：

```yaml
cloud_pub:
  max_z_filter: 1.5
  min_z_filter: -1.0
  enable_body_filter: true
```

当前 aux-height 版本会把高度限制重新加入 ERASOR2 导出。也就是说，导出的 `velodyne/*.bin` 会受到：

```text
min_z_filter <= z <= max_z_filter
```

限制。

如需关闭高度裁剪，可把范围放宽，例如：

```yaml
cloud_pub:
  max_z_filter: 3.0
  min_z_filter: -2.0
```

或者根据实际场地调节。

---

## 4. 运行 Adaptive-LIO 并导出数据

先删除旧数据，避免新旧帧混在一起：

```bash
cd /home/sb/Eraser_for_dynamic/Adaptive-LIO

rm -rf erasor2_dataset
mkdir -p erasor2_dataset
```

编译 Adaptive-LIO：

```bash
colcon build --symlink-install
source install/setup.bash
```

启动 Adaptive-LIO：

```bash
ros2 launch <你的包名> <你的launch文件>.py
```

然后播放 rosbag：

```bash
ros2 bag play <你的rosbag路径>
```

播放结束后，检查输出目录：

```bash
SEQ=/home/sb/Eraser_for_dynamic/Adaptive-LIO/erasor2_dataset/dataset/sequences/00

echo "bin:"   $(ls $SEQ/velodyne/*.bin 2>/dev/null | wc -l)
echo "poses:" $(wc -l < $SEQ/poses_suma_optim.txt)
echo "times:" $(wc -l < $SEQ/times.txt)
```

正常情况下三者数量必须一致，例如：

```text
bin:   2764
poses: 2764
times: 2764
```

如果数量不一致，不要继续跑 ERASOR2，应先检查导出流程。

---

## 5. 生成 dummy SemanticKITTI labels

自采数据没有 SemanticKITTI 官方人工语义标签，但 ERASOR2 的 SemanticKITTI dataloader 会读取：

```text
labels/000000.label
labels/000001.label
...
```

因此需要生成 dummy label。每个 `.label` 文件必须和对应 `.bin` 点数一致，类型为 `uint32`。
在adaptive根目录运行：

```bash
python3 scripts/make_dummy_semantickitti_labels.py \
  /home/sb/Eraser_for_dynamic/Adaptive-LIO/erasor2_dataset/dataset/sequences/00
```

检查：

```bash
SEQ=/home/sb/Eraser_for_dynamic/Adaptive-LIO/erasor2_dataset/dataset/sequences/00

echo "labels:" $(ls $SEQ/labels/*.label 2>/dev/null | wc -l)
```

应与 `velodyne/*.bin` 数量一致。

---

## 6. ERASOR2 环境准备，不使用 conda

进入 ERASOR2：

```bash
cd /home/sb/Eraser_for_dynamic/ERASOR2
```

创建 Python venv：

```bash
python3 -m venv .venv
source .venv/bin/activate

python -m pip install -U pip setuptools wheel
```

安装 Python 依赖：

```bash
pip install \
  numpy \
  open3d \
  hdbscan \
  scikit-learn \
  pypatchworkpp \
  tqdm \
  tabulate \
  PyYAML \
  rerun-sdk
```

如果 `kitti_clustering.py` 报：

```text
AttributeError: module 'numpy' has no attribute 'in1d'
```

说明 NumPy 版本较新。把 ERASOR2 脚本中的 `np.in1d` 改成 `np.isin`：

```bash
cd /home/sb/Eraser_for_dynamic/ERASOR2
sed -i 's/np\.in1d(/np.isin(/g' scripts/pcd_preprocess.py
```

---

## 7. 编译 ERASOR2

安装系统依赖：

```bash
sudo apt update
sudo apt install -y \
  build-essential cmake git \
  libpcl-dev libeigen3-dev libopencv-dev libomp-dev \
  libboost-system-dev libboost-filesystem-dev \
  libyaml-cpp-dev
```

编译：

```bash
cd /home/sb/Eraser_for_dynamic/ERASOR2

cmake -B build -S .
cmake --build build -j$(nproc)
```

如果编译时下载 Arrow / Rerun 失败，通常是代理或 GitHub 网络问题。可以先检查下载：

```bash
curl -L \
  https://github.com/apache/arrow/releases/download/apache-arrow-18.0.0/apache-arrow-18.0.0.tar.gz \
  -o /tmp/apache-arrow-18.0.0.tar.gz
```

如果代理异常，可临时取消代理：

```bash
unset http_proxy
unset https_proxy
unset HTTP_PROXY
unset HTTPS_PROXY
unset all_proxy
unset ALL_PROXY
```

然后重新编译。

---

## 8. 生成 Patchwork ground label 和 HDBSCAN instance label

ERASOR2 运行前必须生成：

```text
patchwork/*.label
hdbscan/*.label
```

先查看最后一帧编号：

```bash
SEQ=/home/sb/Eraser_for_dynamic/Adaptive-LIO/erasor2_dataset/dataset/sequences/00
ls $SEQ/velodyne | tail
```

如果最后一个文件是：

```text
002763.bin
```

则总帧范围是：

```text
0 到 2763
```

运行：

```bash
cd /home/sb/Eraser_for_dynamic/ERASOR2
source .venv/bin/activate

python scripts/kitti_clustering.py \
  --kitti_dir /home/sb/Eraser_for_dynamic/Adaptive-LIO/erasor2_dataset \
  --seq 00 \
  --init_stamp 0 \
  --end_stamp 2763 \
  --save-instance-labels \
  --save-ground-labels
```

注意：`--kitti_dir` 必须是：

```text
/home/sb/Eraser_for_dynamic/Adaptive-LIO/erasor2_dataset
```

而不是：

```text
/home/sb/Eraser_for_dynamic/Adaptive-LIO/erasor2_dataset/dataset/sequences
```

因为脚本内部会自动拼：

```text
<kitti_dir>/dataset/sequences/00/velodyne
```

生成后检查：

```bash
SEQ=/home/sb/Eraser_for_dynamic/Adaptive-LIO/erasor2_dataset/dataset/sequences/00

echo "bin:"       $(ls $SEQ/velodyne/*.bin 2>/dev/null | wc -l)
echo "poses:"     $(wc -l < $SEQ/poses_suma_optim.txt)
echo "times:"     $(wc -l < $SEQ/times.txt)
echo "labels:"    $(ls $SEQ/labels/*.label 2>/dev/null | wc -l)
echo "patchwork:" $(ls $SEQ/patchwork/*.label 2>/dev/null | wc -l)
echo "hdbscan:"   $(ls $SEQ/hdbscan/*.label 2>/dev/null | wc -l)
```

正常应全部一致，例如：

```text
bin:       2764
poses:     2764
times:     2764
labels:    2764
patchwork: 2764
hdbscan:   2764
```

---

## 9. 配置 ERASOR2 的 seq_00.yaml

打开：

```bash
gedit /home/sb/Eraser_for_dynamic/ERASOR2/config/erasor2/seq_00.yaml
```

建议先使用下面的基础配置跑通：

```yaml
start_frame: 0
end_frame: 2763
viz_interval: 10
is_large_scale: false

dataloader:
    run_traj_clustering: false
    dataset_name: "SemanticKITTI"
    abs_data_dir: "/home/sb/Eraser_for_dynamic/Adaptive-LIO/erasor2_dataset/dataset/sequences"
    cloud_dir: ""
    cloud_format: ""
    pose_path: ""
    sequence: "00"
    abs_save_dir: "/home/sb/Eraser_for_dynamic/erasor2_output"
    instance_seg_method: "hdbscan"

    accum_interval: 2
    voxel_size: 0.2
    map_voxel_size: 0.2

    expansion_range: 0

erasor2:
    grid_resolution: 0.5
    egocentric_grid_resolution: 0.3
    range_of_interest: 40.0

    min_z_voi: -1.5
    max_z_voi: 2.3
    min_z_diff_thr: 0.3
    scan_ratio_threshold: 0.2

    log_odds:
        increment_gain: 2.0
        increment: 0.15

    region_proposal_thr: 0.8
    kernel_size: 1

    ratio_num_pts: 0.95
    minimum_num_pts: 5

    moving_object_detection:
        negative_log_odds: -2.0
        obj_score_soft_thr: 4.6
        obj_score_hard_thr: 14.0
        hard_thr_radius: 10.0

    over_segmentation:
        minimum_area_thr: 8
        ratio_of_unknown_prior: 0.25

    volumetric_outlier_removal:
        window_size: 1
        use_adaptive_voxel_size: true
        vor_cand_score_thr: 4.6
        dist_thr_gain: 1.732

    viz_flag:
        set_scan_and_pose: false
        set_submap: false
        update: false
        detect: false
        over_seg: false

    save_map: true

stop_for_each_frame: false

extrinsic:
    robot_body_size: 0.8
    sensor_height: 0.0
    rotation: [ 1, 0, 0,
                0, 1, 0,
                0, 0, 1 ]
    translation: [ 0.0, 0.0, 0.0 ]
```

注意：

```yaml
sensor_height: 0.0
```

建议先设为 0，用于验证几何建图。如果 `mapgen` 输出和 Python 手动累计一致，再根据实际需要调整。

---

## 10. 运行 mapgen 和 run_erasor2

清空旧输出：

```bash
rm -rf /home/sb/Eraser_for_dynamic/erasor2_output
mkdir -p /home/sb/Eraser_for_dynamic/erasor2_output
```

运行 mapgen：

```bash
cd /home/sb/Eraser_for_dynamic/ERASOR2

./build/mapgen config/erasor2/seq_00.yaml
```

运行 ERASOR2：

```bash
./build/run_erasor2 config/erasor2/seq_00.yaml
```

输出文件查找：

```bash
find /home/sb/Eraser_for_dynamic/erasor2_output -type f | sort
find /home/sb/Eraser_for_dynamic/erasor2_output -name "*.pcd" | sort
```

---
## 12. 常见错误与解决

### 12.1 缺 hdbscan label

错误：

```text
Failed to load instance label: .../hdbscan/000000.label
```

原因：

```text
没有运行 kitti_clustering.py，或 hdbscan 目录为空
```

解决：

```bash
python scripts/kitti_clustering.py \
  --kitti_dir /home/sb/Eraser_for_dynamic/Adaptive-LIO/erasor2_dataset \
  --seq 00 \
  --init_stamp 0 \
  --end_stamp 2763 \
  --save-instance-labels \
  --save-ground-labels
```

---

### 12.2 缺 SemanticKITTI label

错误：

```text
File does not exist: .../labels/000000.label
```

原因：

```text
自采数据没有 SemanticKITTI 语义标签
```

解决：

```bash
python3 scripts/make_dummy_semantickitti_labels.py \
  /home/sb/Eraser_for_dynamic/Adaptive-LIO/erasor2_dataset/dataset/sequences/00
```

---

### 12.3 NumPy 没有 in1d

错误：

```text
AttributeError: module 'numpy' has no attribute 'in1d'
```

解决：

```bash
cd /home/sb/Eraser_for_dynamic/ERASOR2
sed -i 's/np\.in1d(/np.isin(/g' scripts/pcd_preprocess.py
```

---

### 12.4 path 和点云互相垂直

原因通常是原版 ERASOR2 把 `poses_suma_optim.txt` 当作 KITTI/SuMa 相机系 pose，又做了相机到雷达转换。

解决：

```text
使用 custom pose fix 版 ERASOR2。
SemanticKITTILoader::loadAllPoses() 必须直接读取 T_map_lidar / T_map_local。
```

日志里应看到类似：

```text
Total xxxx poses are loaded as direct T_map_lidar poses
```

---

### 12.5 地图像时间畸变、墙很厚

优先检查：

```text
1. bin 和 pose 是否同一帧同一时刻
2. 点云是否已经去畸变
3. aux 雷达外参/时间是否准确
4. 高度裁剪是否过窄
5. mapgen 输出是否和 Python 手动累计一致
```

做法：

```text
先只导出主雷达测试
再打开 aux 雷达
最后逐步加高度裁剪
```

当前 aux-height 版本已经把 aux 和高度裁剪加回，但如果地图明显变厚，应重新对比“主雷达-only”和“主+aux”的结果。

---

### 12.6 Rerun Viewer 端口占用

提示：

```text
A process is already listening at this address
addr=0.0.0.0:9876
```

这通常不是致命错误。可以忽略，也可以关闭：

```bash
pkill rerun
```

---

## 13. 推荐完整流程

```bash
# 1. Adaptive-LIO 重新导出 ERASOR2 数据
cd /home/sb/Eraser_for_dynamic/Adaptive-LIO
rm -rf erasor2_dataset
colcon build --symlink-install
source install/setup.bash
ros2 launch <你的包名> <你的launch文件>.py
ros2 bag play <你的rosbag路径>

# 2. 生成 dummy labels
python3 scripts/make_dummy_semantickitti_labels.py \
  /home/sb/Eraser_for_dynamic/Adaptive-LIO/erasor2_dataset/dataset/sequences/00

# 3. 生成 patchwork/hdbscan labels
cd /home/sb/Eraser_for_dynamic/ERASOR2
source .venv/bin/activate
sed -i 's/np\.in1d(/np.isin(/g' scripts/pcd_preprocess.py

python scripts/kitti_clustering.py \
  --kitti_dir /home/sb/Eraser_for_dynamic/Adaptive-LIO/erasor2_dataset \
  --seq 00 \
  --init_stamp 0 \
  --end_stamp <最后一帧编号> \
  --save-instance-labels \
  --save-ground-labels

# 4. 检查数量一致
SEQ=/home/sb/Eraser_for_dynamic/Adaptive-LIO/erasor2_dataset/dataset/sequences/00
echo "bin:"       $(ls $SEQ/velodyne/*.bin 2>/dev/null | wc -l)
echo "poses:"     $(wc -l < $SEQ/poses_suma_optim.txt)
echo "times:"     $(wc -l < $SEQ/times.txt)
echo "labels:"    $(ls $SEQ/labels/*.label 2>/dev/null | wc -l)
echo "patchwork:" $(ls $SEQ/patchwork/*.label 2>/dev/null | wc -l)
echo "hdbscan:"   $(ls $SEQ/hdbscan/*.label 2>/dev/null | wc -l)

# 5. 跑 ERASOR2
rm -rf /home/sb/Eraser_for_dynamic/erasor2_output
mkdir -p /home/sb/Eraser_for_dynamic/erasor2_output

./build/mapgen config/erasor2/seq_00.yaml
./build/run_erasor2 config/erasor2/seq_00.yaml

# 6. 查看输出
find /home/sb/Eraser_for_dynamic/erasor2_output -name "*.pcd" | sort
```

---

## 14. 最重要的检查清单

在跑 `run_erasor2` 前，必须满足：

```text
[ ] velodyne/*.bin 数量正确
[ ] poses_suma_optim.txt 行数正确
[ ] times.txt 行数正确
[ ] labels/*.label 数量正确
[ ] patchwork/*.label 数量正确
[ ] hdbscan/*.label 数量正确
[ ] seq_00.yaml 的 end_frame 等于最后一帧编号
[ ] ERASOR2 使用 custom pose fix 版本
[ ] mapgen 输出和 Python 手动累计大体一致
```

只要这几个条件满足，就可以稳定跑完整 ERASOR2 流程。





























<div align="center">
    <h1>ERASOR2</h1>
    <a href="https://github.com/url-kaist/ERASOR2"><img src="https://img.shields.io/badge/-C++-blue?logo=cplusplus" /></a>
    <a href="https://github.com/url-kaist/ERASOR2"><img src="https://img.shields.io/badge/Python-3670A0?logo=python&logoColor=ffdd54" /></a>
    <a href="https://github.com/url-kaist/ERASOR2"><img src="https://img.shields.io/badge/Ubuntu-20.04%20%7C%2022.04-E95420?logo=ubuntu&logoColor=white" /></a>
    <a href="https://github.com/url-kaist/ERASOR2"><img src="https://img.shields.io/badge/CMake-064F8C?logo=cmake&logoColor=white" /></a>
    <a href="https://www.ipb.uni-bonn.de/wp-content/papercite-data/pdf/lim2023rss.pdf"><img src="https://img.shields.io/badge/Paper-b33737?logo=arXiv" /></a>
    <a href="https://www.youtube.com/watch?v=cELvWYxfrpY"><img src="https://img.shields.io/badge/YouTube-FF0000?logo=youtube&logoColor=white" /></a>
    <a href="https://github.com/url-kaist/ERASOR2"><img src="https://img.shields.io/badge/license-GPLv3-green" /></a>
    <br />
    <br />
    <a href="https://www.youtube.com/watch?v=cELvWYxfrpY">Video</a>
    <span>&nbsp;&nbsp;•&nbsp;&nbsp;</span>
    <a href="#package-installation">Install</a>
    <span>&nbsp;&nbsp;•&nbsp;&nbsp;</span>
    <a href="#rocket-how-to-run">How to Run</a>
    <span>&nbsp;&nbsp;•&nbsp;&nbsp;</span>
    <a href="https://www.ipb.uni-bonn.de/wp-content/papercite-data/pdf/lim2023rss.pdf">Paper</a>
    <span>&nbsp;&nbsp;•&nbsp;&nbsp;</span>
    <a href="https://github.com/url-kaist/ERASOR2/issues">Contact Us</a>
    <br />
    <br />
  <p align="center">
    <img width="640" height="485" alt="ERASOR2 demo" src="https://github.com/user-attachments/assets/5dc13005-c22a-4428-a55e-1f3d6ed97339" />
    <br />
    <img width="640" height="294" alt="ERASOR2 comparison" src="https://github.com/user-attachments/assets/3eb35e63-7b71-4bfd-84cd-74605ed34a90" />
  </p>
  <p><strong><em>ROS-free, instance-aware static map building</em></strong></p>
</div>

______________________________________________________________________

## :package: Installation

```bash
# 1. Build (one cmake call, no ROS/catkin).
cmake -B build -S . && cmake --build build -j

# 2. Conda env for the Python preprocessors + evaluator.
conda env create -f scripts/environment.yml   # creates env "erasor2"
conda activate erasor2
```

See [**USAGE.md**](USAGE.md) for the full dependency list and per-distro
notes.

______________________________________________________________________

## SemanticKITTI Setup

Download SemanticKITTI so the sequence folders live under
`<kitti_dir>/dataset/sequences`. For example, the benchmark tree should
look like this:

```text
<kitti_dir>/                         # e.g., /home/<user id>/datasets/kitti
└── dataset/
    ├── poses/
    └── sequences/
        ├── 00/
        │   ├── velodyne/
        │   ├── labels/
        │   ├── **poses_suma_optim.txt** (important)
        │   └── times.txt
        ├── 01/
        ├── 02/
        ├── ...
        └── 10/
```

ERASOR2 uses SuMa poses for evaluation. Download the pose archive and
place each `poses_suma_optim.txt` inside its matching sequence directory:

```bash
wget -O suma_poses_for_erasor_eval.zip "https://www.dropbox.com/scl/fi/9q3b1b9npsst1zjawgou3/suma_poses_for_erasor_eval.zip?rlkey=vx4igm68iuo3eobpolgq4tblg&st=yt1ola9b&dl=0"
# unzip, then copy each file to:
# <kitti_dir>/dataset/sequences/<seq>/poses_suma_optim.txt
```

For each benchmark config in `config/erasor2/seq_{00,01,02,05,07}.yaml`,
set `dataloader.abs_data_dir` to `<kitti_dir>/dataset/sequences` and
`dataloader.abs_save_dir` to your ERASOR2 output directory.

______________________________________________________________________

## :rocket: How to Run

```bash
# 3. Generate per-frame Patchwork ground + HDBSCAN instance labels
#    for seqs 00, 01, 02, 05, 07 in one shot.
scripts/generate_labels.sh /path/to/kitti

# 4. Edit config/erasor2/seq_{00,01,02,05,07}.yaml to point at your
#    kitti and output directories, then run the full benchmark.
python scripts/run_benchmark.py
```

`scripts/run_benchmark.py` invokes `run_pipeline.py` for each yaml
(mapgen &rarr; run_erasor2 &rarr; evaluate.py), then prints a single
consolidated PR / RR / F1 table. See [**USAGE.md**](USAGE.md) for further
explanation &mdash; per-step breakdown, path-editing conventions,
visualizer, YAML reference, and HeLiPR / HeLiMOS setup.

______________________________________________________________________

## :bar_chart: Headline numbers

Some reproduced numbers may differ slightly from the paper after the
ROS-free refactor, but the overall performance remains consistent with
the reported HDBSCAN-based results. Because this implementation uses
HDBSCAN for instance segmentation, compare against the HDBSCAN rows in
Table III of the paper.

<div align="center">

| Seq | Frames | PR [%] ( $\color{#c026d3}\textsf{paper}$ / $\color{#0969da}\textsf{ours}$ ) | RR [%] ( $\color{#c026d3}\textsf{paper}$ / $\color{#0969da}\textsf{ours}$ ) | F1 ( $\color{#c026d3}\textsf{paper}$ / $\color{#0969da}\textsf{ours}$ ) |
|:---:|:---:|:---:|:---:|:---:|
| 00 | 4390 – 4530 | $\color{#c026d3}98.649$ / $\color{#0969da}\mathbf{98.654}$ | $\color{#c026d3}\mathbf{98.582}$ / $\color{#0969da}98.454$ | $\color{#c026d3}\mathbf{0.986}$ / $\color{#0969da}0.9855$ |
| 01 |  150 –  250 | $\color{#c026d3}93.554$ / $\color{#0969da}\mathbf{95.743}$ | $\color{#c026d3}\mathbf{94.951}$ / $\color{#0969da}94.027$ | $\color{#c026d3}0.943$ / $\color{#0969da}\mathbf{0.9488}$ |
| 02 |  860 –  950 | $\color{#c026d3}98.339$ / $\color{#0969da}\mathbf{99.196}$ | $\color{#c026d3}99.709$ / $\color{#0969da}\mathbf{99.902}$ | $\color{#c026d3}0.990$ / $\color{#0969da}\mathbf{0.9955}$ |
| 05 | 2350 – 2670 | $\color{#c026d3}97.473$ / $\color{#0969da}\mathbf{97.670}$ | $\color{#c026d3}\mathbf{99.113}$ / $\color{#0969da}98.412$ | $\color{#c026d3}\mathbf{0.983}$ / $\color{#0969da}0.9804$ |
| 07 |  630 –  820 | $\color{#c026d3}\mathbf{98.767}$ / $\color{#0969da}96.135$ | $\color{#c026d3}98.800$ / $\color{#0969da}\mathbf{98.989}$ | $\color{#c026d3}\mathbf{0.988}$ / $\color{#0969da}0.9754$ |

</div>

<sub>$\color{#c026d3}\textsf{Magenta}$ = paper (Table III, HDBSCAN row), $\color{#0969da}\textsf{blue}$ = our re-run. **Bold** marks the higher value per cell.</sub>

ERASOR2 reproduces within run-to-run noise (mean |&Delta;F1| = 0.006).
Higher is better on all three metrics:

- **PR (Preservation Rate)** measures how much true
  static structure remains after dynamic-object removal.
- **RR (Rejection Rate)** measures how much dynamic
  structure is correctly rejected from the static map.
- **F1** is the harmonic mean of PR and RR, giving one balanced score
  when preservation and rejection both matter.

______________________________________________________________________

## :books: Citation

If you use this code in academic work, please cite the ERASOR / ERASOR2
papers.

```bibtex
@article{lim2025erasor2,
  title   = {{ERASOR2}: Instance-Aware Robust 3D Mapping of the Static World in Dynamic Scenes},
  author  = {Lim, Hyungtae and others},
  journal = {IEEE Robotics and Automation Letters},
  year    = {2025}
}
```

```bibtex
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

```bibtex
@inproceedings{lim2024helimos,
  title     = {{HeLiMOS: A dataset for moving object segmentation in 3D point clouds from heterogeneous LiDAR sensors}},
  author    = {Lim, Hyungtae and Jang, Seoyeon and Mersch, Benedikt and Behley, Jens and Myung, Hyun and Stachniss, Cyrill},
  booktitle = {2024 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
  pages     = {14087--14094},
  year      = {2024}
}
```
