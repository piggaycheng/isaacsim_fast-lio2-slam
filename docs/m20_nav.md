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
   - 以 LIO + IMU 維持 3D `odom -> base_link`（含 z／roll／pitch，供高程圖與策略對齊高度），並以 3D PCD ICP + 3D Global EKF（`two_d_mode: false`，融合 `z` 與 `vz`）+ 3D `global_tf_gate` 維持包含 Z 軸高度的 `map -> odom`。
   - `global_localization_xyz` 在 3D 模式下會讀取同層與相鄰樓層的 `session.json`，自動將多樓層 PCD（如 `maps/1F/map.pcd` 與 `maps/B1/map.pcd`）轉換並合併至同一 3D 座標系，並以 `fov_z: 2.5 m` 垂直視窗裁切當前樓層／樓梯的局部子圖。
   - 執行 `ground_obstacle_filter`（`publish_obstacles: false`）濾除 M20 機身自體點雲並發布 `/perception/self_filtered_points`，避免平面 RANSAC 將樓梯誤判為障礙物；實際不可跨越障礙物（局部鄰域高低差 `>= 0.30 m`，如牆面、箱子、柱子與樓梯扶手）由 `grid_map_to_points.py` 自 GPU 高程圖提取並發布至 `/perception/obstacles`。
   - 執行 Nav2（`planner_server`、`controller_server`、`direction_zones`、`collision_monitor`、`cmd_vel_safety`），其中 `global_costmap` 與 `local_costmap` 皆採用 `rolling_window: true`（不載入單層靜態 PGM `static_layer`），可直接搭配上位 Open-RMF 跨樓層樓梯 waypoint 連續導航，無須切圖停頓。輪式機器人（`nova_carter`、`carter_v1`）則維持原本的 2D `static_layer` 與電梯切圖流程。
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

---

## 4. 3D 跨樓層樓梯導航模組（已實作，待測試）

為了讓 M20 能走樓梯跨樓層（如 `1F` ↔ `B1`）並由上位 Open-RMF 連續派發 waypoint，同時不影響既有輪式機器人（`nova_carter`、`carter_v1`）的 2D 靜態地圖與電梯切圖導航，本分支針對 `m20` profile 實作了三個獨立模組（目前皆處於**已修改、待實機／模擬驗證**狀態）：

### Module 1：3D 全域定位與多樓層 PCD 地圖支援（Z 軸高度定位）

- **問題**：原本雖然局部里程計（`odom -> base_link`）已是 3D，但全域定位（`map -> odom`）的 `global_pose_adapter`、`global_ekf` 與 `global_tf_gate` 會將 `z`、`roll`、`pitch` 強制歸零（`two_d_mode: true`），且單層 PCD 加上 `fov_z: 2.5 m` 垂直裁切會在上下樓梯時切掉目標樓層點雲。
- **修改內容**：
  1. **3D `map -> odom` 鏈路**：在 `m20.yaml` 啟用 `global_localization.three_d: true`（輪式維持 `false`），讓 `lio_odometry.py`（保留 `vz`）、`global_pose_adapter.py`（保留 ICP 完整 `z` 與 3D 姿態）、`global_ekf`（`two_d_mode: false`）與 `global_tf_gate.py` 全程保留 Z 軸高度。
  2. **多樓層 PCD 自動對齊與合併**：新增 `floor_maps.py` 與 `scripts/merge_floor_pcds.py`。當 M20 啟動 3D 定位時，`global_localization_xyz.py` 可直接載入單一多樓層點雲（如 `--pcd maps/all/map.pcd`），或自動尋找鄰近樓層（如 `maps/1F` 與 `maps/B1`）的 `session.json` 轉換至同一 3D 座標系合併載入，並以 `fov_z: 3.5 m` 的滑動垂直視窗同步裁切 `scan` 與 `submap`（含 `+0.5 m` 邊界裕度），確保在樓梯中段也能穩定收斂。

### Module 2：以 GPU 局部高程圖取代平面 RANSAC 障礙物提取（防止樓梯被誤判為牆）

- **問題**：原本 `ground_obstacle_filter` 使用平面 RANSAC 擬合單一地面，在樓梯口會把整座向上／向下的階梯當成非地面障礙物投射進 `/perception/obstacles`，導致 Nav2 把樓梯封死。
- **修改內容**：
  1. **保留機身濾除、關閉 RANSAC 障礙物輸出**：在 `ground_obstacle_filter.cpp` 新增 `publish_obstacles` 參數，`m20` 設為 `false`，僅輸出濾除機身後的 `/perception/self_filtered_points` 給 `elevation_mapping_cupy`。
  2. **濾除平地與樓梯間斜天花板**：在 `elevation_m20_nav.yaml` 與 `elevation_isaac.yaml` 設定 `max_height_range: 0.70`、`ramped_height_range_a: 0.22`、`ramped_height_range_b: 1.5`、`ramped_height_range_c: 0.15`，防止遠方天花板或樓梯間斜頂被寫入 2.5D 高程圖。
  3. **由高程圖提取陡峭高度差障礙物**：在 `grid_map_to_points.py` 中對 `elevation_mapping_cupy` 輸出的 `elevation` 圖層計算局部鄰域（半徑 `0.12 m`）最大高度差，僅將局部台階高度差 `>= 0.30 m` 且高度在 `base_link` `[-2.0, 0.6] m` 內的格點（牆壁、柱子、箱子、樓梯扶手與邊緣落差）轉換為 `/perception/obstacles` 點雲；一般樓梯單階高度（約 `0.15 ~ 0.20 m`）則視為可通行地形，不會標記為障礙物。

### Module 3：M20 動態滾動 `global_costmap`（免切圖跨樓層連續導航）

- **問題**：原本 `global_costmap` 綁定單一樓層的 2D 靜態 `map_2d.pgm`（`static_layer`），跨樓層必須透過 MQTT `change_floor` 切換地圖並重新定位，造成導航停頓。
- **修改內容**：
  1. **`global_costmap` 改為 `rolling_window: true`**：在 `m20.yaml` 設定 `global_costmap` 為 `24 m × 24 m`（半徑 `±12 m`）、解析度 `0.05 m` 的動態滾動視窗，移除 `static_layer`，僅保留接收 `/perception/obstacles` 與 `/scan`（`max_height: 0.5 m`）的 `obstacle_layer` 與 `inflation_layer`。
  2. **相容上位 Open-RMF Waypoint 流程**：搭配 Open-RMF 分段下發「1F 走廊 → 1F 樓梯口 → B1 樓梯口 → B1 目標點」的局部 waypoint，M20 可全程在同一個連續 3D `map` 座標系下完成跨樓層導航，不需切圖或重設初始位姿。

---

## 5. 使用 `run_nav_m20.sh --free-fly` 無策略（No-Policy）驗證模組 1、2、3

若目前手邊尚無訓練好的 M20 `policy.pt`，`scripts/m20_standalone.py` 與 `scripts/run_nav_m20.sh` 已內建 `--free-fly` 無車體感測載具模式（離地高度固定維持與 M20 相同的 `STANDING_HEIGHT = 0.49 m`），**可直接接收 Nav2 `/cmd_vel` 全自動導航，並在經過樓梯時透過向下 PhysX Raycast 自動隨著階梯抬升／下降**。

### 5.1 各模組支援度評估

| 測試項目 | 是否可用 `run_nav_m20.sh --free-fly` 測試？ | 原因與行為說明 |
|---|---|---|
| **Module 1：3D 定位與跨樓層 Z 軸追蹤** | **完全可以（100%）** | `m20_standalone.py --free-fly` 在 `/World/M20/base_link` 建立輕量感測載具，並掛載與 M20 相同外參（`[0.2, 0.0, 0.15]`）的 RTX LiDAR 與 IMU。上下樓梯時，載具透過向下 Raycast 偵測 Office 樓梯網格高度，自動維持離地 `0.49 m`，可在 RViz 驗證 `map -> odom -> base_link` 的 Z 軸高度變化與多樓層 PCD ICP 收斂。 |
| **Module 2：高程圖樓梯可通行性與障礙物提取** | **完全可以（100%）** | `run_nav_m20.sh --free-fly` 在 `height_map` 容器中只啟動 `isaac_fastlio_adapter/height_map.launch.py`（不需啟動 `m20_policy_node`）。載具以 `0.49 m` 高度靠近與爬升樓梯時，可在 RViz 觀察 `/elevation_mapping_node/elevation_map_filter` 與 `/perception/obstacles` 是否放行樓梯階面、並正確標出兩側扶手與牆壁。 |
| **Module 3：Rolling `global_costmap` 與 Nav2 全自動跨樓層導航** | **完全可以（100% 閉迴路導航）** | `m20_standalone.py --free-fly` 透過 OmniGraph `ROS2SubscribeTwist` 訂閱 Nav2 輸出的 `/cmd_vel` 自動移動，同時在遇到樓梯時自動隨階梯升降（無需人工按鍵，但仍支援隨時以 `W/S/A/D` 或方向鍵、`E/Q` 或 `PageUp/PageDown` 手動介入）。 |

### 5.2 無 `policy.pt` 時的一鍵啟動與測試步驟

直接執行：

```bash
./scripts/run_nav_m20.sh --free-fly
```

啟動後驗證流程：
1. 在導航 RViz 點選 **2D Pose Estimate** 完成初始定位。
2. 在導航 RViz 點選 **2D Goal Pose**（或由上位 Open-RMF 下發 waypoint）指向樓梯口及下一層樓目標點：
   - **檢查 Module 1**：觀察感測載具沿樓梯自動爬升／下降（維持離階面 `0.49 m`），且 RViz 中 `base_link` TF 的 Z 軸高度平滑升降、跨樓層 ICP 穩定匹配。
   - **檢查 Module 2**：觀察 `/perception/obstacles` 與 Costmap 在樓梯踏階處保持淨空，僅在牆面與欄杆處產生障礙點。
   - **檢查 Module 3**：確認 `global_costmap` 隨機器人位置滾動更新，且 Nav2 能全程透過 `/cmd_vel` 驅動載具穿過樓梯完成跨樓層導航。

