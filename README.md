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
docker compose run --rm ros build --packages-select slam_localization_3d
```

導航與定位套件使用中性名稱：`slam_nav`、`slam_localization_2d`、`slam_localization_3d`。`isaac_fastlio_adapter` 仍是 Isaac Sim 專用的感測器轉接套件；目前 launch 與預設配置仍針對模擬，改名不代表可直接套用到實機。從舊名稱升級的工作區，需移除這三個舊套件對應的 `ros2_ws/build/` 與 `ros2_ws/install/` 子目錄後重新建置，避免舊套件仍被 ROS 找到。

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
./scripts/run_multi_nav.sh                     # carter1：Nova Carter；carter2：Carter v1
./scripts/run_multi_nav.sh --robot carter1@0,0 --robot carter2:carter_v1@3.5,0,1.57
./scripts/run_multi_nav.sh --robot a@0,0 --robot b@3.5,-2,1.57 --robot c@1,2
```

`--robot NAME[:TYPE]@X,Y[,YAW]` 可重複指定。位置單位為公尺、航向為弧度；自行指定 `--robot` 時，省略 TYPE 仍代表 `nova_carter`。不指定任何 `--robot` 時，第二台預設使用 `carter_v1` 的獨立 profile（LiDAR 安裝與待校正參數見[多車文件](docs/multi_robot.md#carter-v1)）。每車各有一個導航 container；RViz 的 **Fleet Control** 下拉選單選車，再按 **Set navigation goal** 並在地圖上拖曳。

多車時 Isaac viewport 預設固定俯視，不跟隨第一台車。支援 `--map`、`--pcd`、`--headless`、`--no-rviz` 等參數。**目前沒有車輛間的路權／交通協調**，狹窄通道可能互相卡住。

各腳本完整參數可用 `--help` 查看；`scripts/run_slam.sh` 沒有參數介面。

### 內部腳本

| 腳本 | 用途與範例 |
|---|---|
| `docker/build_workspace.sh` | 容器內建置流程，通常透過 `docker compose run --rm ros build` 使用 |
| `docker/entrypoint.sh` | 容器入口，載入 ROS 環境並執行命令，不需從主機直接啟動 |
| `docker/ros_compose.sh` | 啟動腳本共用的 Compose helper，供其他腳本 `source` 使用 |

## 3. 更換車種：footprint 與 covariance

換成非 Nova Carter 的車種時，不能直接沿用 Carter 的車體尺寸、輪徑、輪距、感測器外參與 covariance。以 `ros2_ws/src/slam_localization_3d/config/robots/nova_carter.yaml` 為範本複製成 `<type>.yaml`，設定 `simulation`、`sensor_frames` 與 `parameter_overrides`；Carter profile 已列出所有車種相關參數（輪子、covariance、footprint、self filter、安全區域與速度），新車逐項換成自己的值；多車入口可用 `--robot NAME:<type>@X,Y` 選擇新車種，設定格式見[新增車種](docs/multi_robot.md#新增車種)。

| 工具 | 用途 |
|---|---|
| `scripts/usd_bbox.py` | 從 USD 模型量測車體 bounding box，輸出 Nav2 footprint |
| `slam_localization_3d` 的 `covariance_drive.py` | 發布校正路線的速度指令，收集靜止、直行與旋轉樣本；**不是計算 covariance 的工具** |
| `ros2_ws/src/slam_localization_3d/scripts/covariance_calibration.py` | 離線分析 rosbag，估計輪速、IMU 與 PCD 定位的 covariance |

### 量測 footprint

在主機使用 **Isaac Sim 的 Python launcher**，將下列路徑與 frame 換成新車的模型及 ROS `base_link` 對應的 USD prim：

```bash
isaac_python=/path/to/isaacsim/python.sh
"$isaac_python" scripts/usd_bbox.py /path/to/new_robot.usd \
  --frame base_link --yaw-deg 0 --padding 0.06
```

`--yaw-deg` 必須符合 USD frame 與 ROS `base_link` 的方向差，**不要直接套用 Carter 的 180 度**。`--padding 0.06` 只是示例，不是所有車種都適用的安全距離；`--shape hull` 可改為凸包，`--json` 可輸出完整量測結果。沒有 USD 模型的實機，需以實測尺寸或可信的車體模型建立 footprint。

將輸出的 footprint 同步填入 global／local costmap（`config/observation_costmaps.yaml`），並調整 `slam_nav/config/ground_obstacle_filter.yaml` 的 `self_filter_bounds`，以及 `config/collision_monitor.yaml` 的停止／減速區域。以上相對路徑的 `config/` 位於 `slam_localization_3d`；新車種應寫在自己 profile 的 `parameter_overrides`（對應鍵見 `nova_carter.yaml`），不要改動共用預設值。工具**不會自動寫入設定，也不會量測煞停距離**；修改後仍需驗證車體包絡與碰撞停止行為。

### 校正 covariance（協方差）

先確認新車的輪子幾何與感測器外參正確，並啟動定位、關閉 Nav2 自主導航。以下是 **ROS 容器已啟動、使用無 namespace topics** 時的範例；錄製 bag 與駕駛需在不同終端執行：

```bash
# 終端 1：錄製；校正路線完成後按 Ctrl+C 停止錄製
docker compose exec ros /workspace/docker/entrypoint.sh \
  ros2 bag record -o cov_bag \
  /wheel/odom /nav/imu /Odometry /localization_3d/global_pose

# 終端 2：模擬環境中的校正路線；也可改用手動駕駛收集樣本
docker compose exec ros /workspace/docker/entrypoint.sh \
  ros2 run slam_localization_3d covariance_drive.py --ros-args -p use_sim_time:=true

# 錄製完成後，離線計算建議值；預設不修改 YAML
docker compose run --rm ros \
  python3 ros2_ws/src/slam_localization_3d/scripts/covariance_calibration.py cov_bag
```

**`covariance_drive.py` 會直接發布 `/cmd_vel`，不經 Nav2 或碰撞檢查。** 實機應改用 `use_sim_time:=false`，先確認指令接到正確底盤、準備足夠淨空與急停；不要直接照模擬範例讓真車行駛。

LIO body 到 `base_link` 的轉換預設由 `--robot-type` profile 的 `sensor_frames.imu_link` 反推；要測試其他值時可用 `--lio-body-to-base X Y Z YAW`（公尺／弧度）覆寫。若有 namespace，錄製時改用該車的完整 topic，分析時透過 `--wheel-topic`、`--imu-topic`、`--lio-topic`、`--pcd-topic` 指定；駕駛節點也需 remap `/cmd_vel`。以 `--robot-type <type>` 指定車種（預設 `nova_carter`），工具會讀取該 profile 中錄製當下使用的 covariance floor。

確認建議值後加 `--apply`，結果只會寫入該車種 profile 的 `parameter_overrides`；共用的 `local_odometry.yaml`、`global_fusion.yaml` 不含輪速、IMU、PCD covariance 等車種相關值（2D 模式也從 Nova Carter profile 讀取）。重新建置並重啟後再驗證，完整流程、輸出判讀與限制見[協方差校正](docs/covariance_calibration.md)。

## 詳細文件

- [ROS 工作區整合與操作手冊](ros2_ws/README.md)
- [建圖流程](docs/mapping_flow.md)
- [3D 定位架構](docs/3d_localization.md)
- [導航、安全限制與驗證](docs/nav.md)
- [多車架構、車種設定與部署](docs/multi_robot.md)
- [更換車輛後的協方差校正](docs/covariance_calibration.md)
