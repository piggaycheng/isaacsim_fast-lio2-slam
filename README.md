# Isaac Sim + FAST-LIO2 + Nav2

使用 Isaac Sim 模擬 Nova Carter，支援 FAST-LIO2 建圖、2D／3D 定位與 Nav2 導航。多車模式採用**一車一個導航 container，以及一個共用 RViz**。

## 1. 建置專案環境

### 環境需求

- Linux、NVIDIA GPU 與可用的 NVIDIA 驅動。
- Docker、Docker Compose v2、NVIDIA Container Toolkit。
- Isaac Sim 安裝在 `/home/user/isaacsim-6.1.0`；目前腳本使用此路徑，若安裝位置不同，需調整 `scripts/standalone.py` 的 shebang 與測試腳本中的 `isaac_python`。
- GUI 模式需要可用的 X11／XWayland 顯示環境及 `DISPLAY`；若使用 Xauthority 驗證，需設定 `XAUTHORITY` 指向可用的授權檔。
- Isaac Sim 的 Office／Nova Carter 資產需可存取。

Isaac Sim 在主機執行，ROS 2 Humble、FAST-LIO2、Nav2 和 RViz 在 Docker 內執行，**主機不必另外安裝 ROS 2**。

```bash
git clone --recurse-submodules https://github.com/piggaycheng/isaacsim_fast-lio2-slam.git
cd isaacsim_fast-lio2-slam

# 已 clone 的專案也可用這行補齊 submodule
git submodule update --init --recursive

export HOST_UID="$(id -u)"
export HOST_GID="$(id -g)"
docker compose build
docker compose run --rm ros build
```

修改 `ros2_ws/src` 後重新執行 workspace build 即可；修改 Docker 安裝依賴時，才需要重建 image：

```bash
docker compose run --rm ros build
# 只建置指定套件
docker compose run --rm ros build --packages-select isaac_localization_3d
```

## 2. 各個 sh 如何使用

專案級啟動腳本與 Python 工具集中在 `scripts/`；ROS 套件、`docker/` 與 `tests/` 內的腳本維持原位。以下指令皆在**專案根目錄**執行，地圖等資料仍放在 `maps/`。一般啟動腳本會自動啟動 ROS container、RViz 與 Isaac Sim，按 **Ctrl+C** 結束並清理 container。除多車模式外，不要同時啟動多套建圖／定位／導航腳本。

### 建圖與存圖

```bash
./scripts/run_slam.sh
```

啟動 FAST-LIO2、PGO 回環與 RViz。使用 **W/S/A/D 或方向鍵**操控車輛，**Space** 停車。

建圖期間保持上述程式運行，另開終端存圖：

```bash
./scripts/save_map.sh
./scripts/save_map.sh --output-dir maps/office --voxel-size 0.05 --save-patches true
```

預設輸出到 `maps/office`，包含 `map.pcd`、`poses.txt` 與 keyframe `patches/`。導航所需的 2D 地圖可再由 PCD 轉換：

```bash
./scripts/pcd2pgm.py
```

### 單車定位與導航

| 腳本 | 用途與範例 |
|---|---|
| `scripts/run_nav.sh` | 統一入口，必須指定 `--mode 2d` 或 `--mode 3d` |
| `scripts/run_2d_localization.sh` | 2D AMCL 定位；例如 `./scripts/run_2d_localization.sh --map maps/office/map_2d.yaml` |
| `scripts/run_3d_localization.sh` | 3D ICP 定位顯示；加 `--global-fusion` 啟動輪速／IMU／PCD 融合 |

```bash
./scripts/run_nav.sh --mode 2d                 # AMCL 定位，不啟動自主導航
./scripts/run_nav.sh --mode 3d                 # PCD 融合定位與 costmap 觀察，不啟動自主導航
./scripts/run_nav.sh --mode 3d --navigate      # 完整 Nav2 導航
./scripts/run_3d_localization.sh              # 獨立的 3D 定位顯示
```

導航堆疊啟動完成後，用 RViz 的 **2D Goal Pose** 在地圖上按住左鍵拖曳，設定目標位置與航向。

常用參數：

- `--headless`：關閉 Isaac GUI；`--no-rviz`：關閉 RViz，兩者可一起使用。
- `--map FILE`：指定地圖 YAML，不是直接指定 PGM；直接使用 `scripts/run_3d_localization.sh` 時改用 `--pgm FILE`。
- `--pcd FILE`：指定 3D PCD 地圖（3D 模式）。
- `--manual-initial-pose`：改用 RViz 的 **2D Pose Estimate** 手動初始化（3D 模式）。
- `--box X,Y[,SX,SY,SZ]`：加入靜態箱子障礙物。
- `--filter-editor`：RViz Keepout／Speed 區域標註，例如 `./scripts/run_nav.sh --mode 3d --navigate --filter-editor`。

### 多車導航

```bash
./scripts/run_multi_nav.sh                     # 預設生成兩台 Nova Carter
./scripts/run_multi_nav.sh --robot carter1@0,0 --robot carter2@3.5,0,1.57
./scripts/run_multi_nav.sh --robot a@0,0 --robot b@3.5,-2,1.57 --robot c@1,2
```

`--robot NAME[:TYPE]@X,Y[,YAW]` 可重複指定。位置單位為公尺、航向為弧度，車種預設 `nova_carter`。每車各有一個導航 container；RViz 的 **Fleet Control** 下拉選單選車，再按 **Set navigation goal** 並在地圖上拖曳。

多車時 Isaac viewport 預設固定俯視，不跟隨第一台車。支援 `--map`、`--pcd`、`--headless`、`--no-rviz` 等參數。**目前沒有車輛間的路權／交通協調**，狹窄通道可能互相卡住。

各腳本完整參數可用 `--help` 查看；`scripts/run_slam.sh` 沒有參數介面。

### 內部腳本

| 腳本 | 用途與範例 |
|---|---|
| `docker/build_workspace.sh` | 容器內建置流程，通常透過 `docker compose run --rm ros build` 使用 |
| `docker/entrypoint.sh` | 容器入口，載入 ROS 環境並執行命令，不需從主機直接啟動 |
| `docker/ros_compose.sh` | 啟動腳本共用的 Compose helper，供其他腳本 `source` 使用 |

## 詳細文件

- [ROS 工作區整合與操作手冊](ros2_ws/README.md)
- [建圖流程](docs/mapping_flow.md)
- [3D 定位架構](docs/3d_localization.md)
- [導航、安全限制與驗證](docs/nav.md)
- [多車架構、車種設定與部署](docs/multi_robot.md)
