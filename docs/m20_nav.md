# Deep Robotics M20 導航與地形高程圖整合

本文件說明如何在本專案中啟動 Deep Robotics M20 輪足機器人，結合 **3D FAST-LIO2 + Nav2 導航**、**`elevation_mapping_cupy` GPU 地形高程圖**與 **RSL-RL 強化學習運動控制策略（`m20_policy_node`）**。

---

## 1. 架構概觀

M20 為 12 個腿部關節 + 4 個輪子的輪足機器人，不能直接以差速輪運動學驅動，而是需要 50 Hz 的策略網路（TorchScript `policy.pt`）根據本體狀態、速度命令 `/cmd_vel` 與周圍地形高程掃描（Height Scan，187 點）輸出 16 個關節目標。

執行時由 **主機端 Isaac Sim** 與 **兩個 Docker Compose 容器（`ros` 與 `height_map`）** 協同運作：

1. **主機（Isaac Sim `scripts/m20_standalone.py`）**：
   - 載入 Office 場景與 M20 USD 資產。
   - 發布 `/clock`、3D LiDAR（`/isaac/lidar_points`）、底盤 IMU（`/isaac/imu`）、M20 策略所需的關節與機身狀態（`/m20/joint_states`、`/m20/imu`、`/m20/base_velocity`）。
   - 訂閱 `/m20/joint_commands` 驅動 12 腿關節位置與 4 輪轉速。
2. **`ros` 容器（ROS 2 Humble，定位與 Nav2 導航）**：
   - 執行 `slam_localization_3d/global_fusion.launch.py`（`robot_type:=m20`）。
   - 以 LIO + IMU 維持 3D `odom -> base_link`（含 z／roll／pitch，供高程圖與策略對齊高度），並以 3D PCD ICP + Global EKF 維持 `map -> odom`。
   - 執行 `ground_obstacle_filter` 濾除 M20 機身自體點雲，發布 `/perception/self_filtered_points` 與 `/perception/obstacles`。
   - 執行 Nav2（`planner_server`、`controller_server`、`direction_zones`、`collision_monitor`、`cmd_vel_safety`），將導航速度命令發布至 `/cmd_vel`。
3. **`height_map` 容器（ROS 2 Humble + CUDA 12.6 + PyTorch + CuPy）**：
   - 執行 `elevation_mapping_cupy`：訂閱 `/perception/self_filtered_points` 與 3D TF（`odom -> base_link`），在 GPU 上即時維護局部高程圖 `/elevation_mapping_node/elevation_map_filter`，並啟動獨立的 `height_map_rviz`（`config/height_map.rviz`，以 `base_link` 為參考系顯示 `GridMap` 表面，方便未來在多機 `multi_nav` 環境下每台 M20 各自開啟獨立的局部高程圖視窗）。
   - 執行 `m20_policy_node`：訂閱 `/cmd_vel`、`/m20/*` 狀態與高程圖，以 50 Hz 推論並發布 `/m20/joint_commands`。

---

## 2. 首次建置環境

確認已初始化 `elevation_mapping_cupy` submodule，並建置 Docker image 與 ROS 2 工作區：

```bash
git submodule update --init --recursive

export HOST_UID="$(id -u)"
export HOST_GID="$(id -g)"

# 建置 ros 與 height_map 兩個映像檔
docker compose --profile height_map build

# 建置工作區（包含 FAST-LIO2、Nav2、elevation_mapping_cupy、m20_policy_node）
docker compose --profile height_map run --rm height_map build
```

---

## 3. 啟動指令

### 3.1 Nav2 導航控制模式（`run_nav_m20.sh`）

使用 Nav2 規劃路徑並輸出 `/cmd_vel` 給 M20 策略控制機器人行走：

```bash
./scripts/run_nav_m20.sh \
  --policy "$HOME/path/to/policy.pt" \
  --policy-device cuda
```

啟動後的操作流程：

1. 等待 Isaac Sim 載入場景，此時會開啟兩個 RViz 視窗：
   - **導航 RViz（`global_fusion.rviz`）**：顯示 2D/3D 全域地圖、定位與 Nav2 路徑。
   - **高程圖 RViz（`height_map.rviz`）**：由 `height_map` 容器啟動，以機器人 `base_link` 為視角顯示該 M20 專屬的 `GridMap` 局部地形高程圖。
2. **手動給定初始定位**：在導航 RViz 工具列點選 **2D Pose Estimate**，在地圖上機器人起點（預設生成於原點 `(0, 0)` 附近）按住左鍵拖曳設定初始位置與航向。
3. 等待終端機顯示 `Initial localization succeeded` 以及 `Nav2 planner, controller, ... and navigator active`。
4. **發送導航目標**：在導航 RViz 工具列點選 **2D Goal Pose**，在地圖上點選目標位置與方向，M20 即會沿規劃路徑行走。

常用參數：

| 參數 | 說明 |
|---|---|
| `--policy FILE` | **必填**，匯出的 TorchScript `policy.pt` 路徑 |
| `--policy-device {cpu\|cuda}` | `m20_policy_node` 推論裝置（預設 `cpu`，建議設 `cuda`） |
| `-m, --map FILE` | 指定 2D 導航地圖 YAML（預設 `maps/1F/map_2d.yaml`） |
| `--pcd FILE` | 指定 3D ICP 用 PCD 地圖（預設為 `--map` 同目錄下的 `map.pcd`） |
| `--build` | 啟動前先在 `height_map` 容器內執行 `colcon build` |
| `--flat-height-scan` | 除錯用：忽略高程圖，固定餵平地高度給策略 |
| `--box X,Y[,SX,SY,SZ]` | 在 Isaac Sim 場景放置靜態方塊障礙物（可重複指定） |
| `--headless` | 不顯示 Isaac Sim GUI |
| `--no-rviz` | 不啟動 RViz |

---

### 3.2 手動 Jog 遙控模式（`run_m20_policy.sh`）

若不跑全域地圖定位與 Nav2，只測試 FAST-LIO2 局部里程計 + GPU 高程圖 + 鍵盤／GUI 遙控 M20（可走樓梯）：

```bash
./scripts/run_m20_policy.sh \
  --policy "$HOME/path/to/policy.pt" \
  --policy-device cuda
```

- 會開啟 `m20_jog` 控制面板與獨立的高程圖 RViz（`config/height_map.rviz`），可直接按鈕或用鍵盤 **W/S/A/D/Q/E** 控制 M20 移動，不需要給定初始定位。
