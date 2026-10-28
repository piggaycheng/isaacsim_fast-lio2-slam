# Plan：四足機器人使用 3D TF tree（供 elevation_mapping_cupy 產生 height scan）

## 目標與範圍

- 四足機器人部署時，RL 步態策略的觀測包含 height scan（Isaac Lab 訓練時由 RayCaster 產生）。
  部署端以 elevation_mapping_cupy 建立局部高程圖，再取樣出相同格式的 height scan。
- 為此需要 `odom → base_link → lidar_link` 帶有正確的 z / roll / pitch。
- **3D TF 只在四足 robot profile 啟用（opt-in）**；現有輪式機器人（`nova_carter`、`carter_v1`）維持 2D，行為完全不變。
- Nav2 對四足仍做平面規劃與控制（x / y / yaw）。

## 現況

| TF | 發布者 | 目前行為 |
|---|---|---|
| `map → odom` | `slam_localization_3d/scripts/global_tf_gate.py` | `map_to_odom()` 只算 x / y / yaw，z 與 roll / pitch 恆為 0 |
| `odom → base_link` | `local_ekf`（robot_localization） | `two_d_mode: true`（`slam_nav/config/local_odometry.yaml:47`） |
| `base_link → lidar_link / imu_link` | static_transform_publisher（robot profile `sensor_frames`） | 已是 3D |
| `camera_init → body` | FAST-LIO（`FAST_LIO_LOCALIZATION2/src/laserMapping.cpp:597-608`） | 完整 3D，與上面的樹不相連 |

- local_ekf 的輸入由 robot profile `local_odometry.inputs` 選擇（`wheel` / `imu` / `lio`，見 `slam_nav/config/local_ekf_inputs.yaml`）。
  只有 `lio`（`lio_odometry.py` → `/lio/odom`）提供 z 位置；`imu` 目前只融合 yaw 角速度。
- 每層樓地圖的 `map` 原點在 IMU 高度，輪式機器人的 `base_link` 在地圖中 z ≈ −0.526（`floor_manager.py:55-56`）。
- Nav2 `ObstacleLayer` 把點雲轉到 `global_frame`（global=`map`、local=`odom`）後，
  用 `min_obstacle_height`（預設 0.0）/ `max_obstacle_height` 過濾 **該座標系的 z**，不是相對車體的高度。
- 換樓層流程（`floor_manager.py` `stop_for_floor_change`）只 hold ICP、global_pose_adapter、global TF、safety 並暫停 Nav2；
  **FAST-LIO 與 local_ekf 持續運作，`odom` 不會重置**。

## 釐清的問題

### Q1. 搭電梯前暫停定位與導航，z 就不會變大？

不會因此歸零：暫停的只有全域定位（ICP、`map → odom`）與 Nav2，`odom → base_link` 仍由 FAST-LIO + local_ekf 連續追蹤。

- 電梯中 LiDAR 看到的車廂與機器人一起移動，FAST-LIO 容易退化：IMU 感受到加速度，但點雲比對認為沒動，
  抵達後 `odom` 的 z 可能是 0、樓高，或介於兩者之間的錯誤值，**無法預期**。
- 就算不搭電梯，四足步態本身會讓 base 高度持續起伏，上下樓梯或坡道也會改變 z。
- 全域 costmap 使用 `map` 座標：輪式機器人在地圖中 base z 已是 −0.526，四足也會依其安裝高度出現非零 z。

所以 costmap 高度過濾（D4）仍要處理；另外換樓層後應清除高程圖（步驟 6）。

### Q2. IMU 有 z 軸加速度，為何說沒有 z 觀測？

更精確地說：**IMU 沒有 z「位置」的觀測**。

- z 加速度要積分兩次才得到高度，零偏與雜訊會讓誤差隨時間平方成長，幾秒內就漂到不可用，需要 LIO 等絕對量測修正。
- IMU **可以**觀測 roll / pitch：靜止或低加速度時，重力方向就是姿態參考。四足應在 local_ekf 中融合 IMU 的 roll / pitch（高頻），
  z 則由 LIO 提供。

## 設計決策

### D1. 由 robot profile 開關 3D TF

- 新增 profile 欄位，例如 `localization: {tf_3d: true}`，預設 `false`。
- launch 依此欄位產生 local_ekf 的 `two_d_mode`、輸入設定，以及 `global_tf_gate` 的模式。
- `tf_3d: true` 時，`local_odometry.inputs` 必須包含 `lio`，否則 launch 報錯。

### D2. local_ekf（`odom → base_link`）3D

- 四足沒有輪速計，輸入為 `lio` + `imu`（之後可加入腿式里程計）。
- `lio`：融合 x, y, z, roll, pitch, yaw（絕對位姿）。
- `imu`：融合 roll, pitch 姿態與三軸角速度；必要時加入加速度。
- **這一段是 height scan 的關鍵**：elevation_mapping_cupy 以 `odom` 建圖時，只依賴 `odom → base_link → lidar_link`。

### D3. `map → odom` 3D（次要）

- height scan 不需要它；主要用途是 RViz 與全域座標一致。可放在第二階段。
- 方式：x / y / yaw 仍取自 global_ekf（維持 2D），z / roll / pitch 取自最新 accepted ICP 修正，
  組成 `T_map_base` 後計算 `T_map_odom = T_map_base · T_odom_base⁻¹`（完整 SE(3)）。
- 第一階段可先維持平面 `map → odom`；`map` 中的 base z 會等於 `odom` 中的 z。

### D4. costmap 高度過濾移到 `base_link`

- `ground_obstacle_filter` 已在 `base_link` 中過濾 0.06–2.0 m。
- 四足 profile 的 costmap `obstacles` / `scan_clearing` 將 `min_obstacle_height` / `max_obstacle_height` 放寬（例如 −100 / 100），
  避免障礙物因全域 z 不同而被丟棄。
- `ground_obstacle_filter` 的地面高度與 self filter 範圍需改用四足的尺寸。

## 實作步驟

### 第一階段：`odom → base_link` 3D + 高程圖

1. **profile 與 launch**
   - `launch/robot_fleet.py`：讀取 `localization.tf_3d`，驗證 `lio` 輸入；預設 `false`。
   - `launch/global_fusion.launch.py`、`robot_namespace.py`：依旗標覆寫 local_ekf `two_d_mode` 與輸入設定。
   - 新增四足 robot profile（模擬 asset、`sensor_frames`、footprint、速度、共變異數）。
2. **local_ekf 輸入模板**
   - `slam_nav/config/local_ekf_inputs.yaml`：新增 3D 版本的 `lio` / `imu` 區塊（或用 `when` 條件依旗標切換）。
   - `nav_imu_adapter`：確認 orientation 共變異數適合融合 roll / pitch。
3. **`lio_odometry.py`**
   - `pose_covariance()` 目前把 z / roll / pitch 設為 `UNUSED_VARIANCE`；新增 `z_variance`、`roll_pitch_variance` 參數。
   - `lio/twist` 維持平面（只給 global_ekf 使用）。
4. **costmap**：四足 profile 的 `observation_costmaps` 覆寫（D4）。
5. **elevation_mapping_cupy 整合**
   - 該 repo `main` 是 ROS1（rospy / `tf::TransformListener`），需使用 ROS2 分支（例如 `dev/ioannis/ros2`）或自行移植。
   - 設定 `map_frame: odom`、`base_frame: base_link`；點雲使用 `/isaac/lidar_points`（`lidar_link`）。
   - 新增 height scan 取樣節點：依 Isaac Lab 訓練設定（格點大小、解析度、yaw-only 對齊、offset），
     以 base 位置取樣高程圖，計算 `base_z − elevation − offset`，並處理未觀測格點的填值方式。
6. **換樓層**：`floor_manager` 在 prepare / 抵達時呼叫 elevation_mapping 的清圖服務，避免電梯中的錯誤高度殘留。
7. **測試**
   - launch 測試：`tf_3d: true` 缺少 `lio` 時失敗；輪式 profile 產生的設定與目前完全相同。
   - `slam_localization_3d/test/test_lio_odometry.py`：新的共變異數欄位。
   - 輪式機器人回歸：`tests/validate_navigation_environment.py`、`tests/run_multi_robot_navigation.sh`、`tests/test_floor_switching.py`。
   - 四足：在 Isaac Sim 中比對 `odom → base_link` z / roll / pitch 與 ground truth、比對 height scan 與 RayCaster 輸出。

### 第二階段：`map → odom` 3D（選做）

8. **`global_tf_gate.py`**：3D 模式時訂閱 ICP 修正，改為完整 SE(3) 組合；hold / stale 行為不變；z / pitch 可低通。
9. **`global_pose_adapter.py`**：jump gate 加入 z 檢查；必要時發布 3D base pose。
10. **測試**：`tests/test_global_tf_gate.py` 新增 3D 與換樓層 z 案例。
11. **文件**：更新 `docs/nav.md`（TF 說明、`tf_3d` 旗標、`lio` 需求）。

## 影響與副作用

### 輪式機器人

- profile 預設 `tf_3d: false`，設定與行為不變；需以回歸測試確認。

### 四足機器人

| 項目 | 原因 | 若不處理的後果 | 對策 |
|---|---|---|---|
| **costmap 障礙物消失** | ObstacleLayer 以 `map` / `odom` 的 z 過濾；base z 不為 0，且隨步態、樓梯、電梯變化 | 低矮或全部障礙物被丟棄，`scan_clearing` 也失效 → **安全問題** | D4，並加測試 |
| 本地里程計依賴 FAST-LIO | z 只能由 LIO 提供 | LIO 發散時 `odom → base_link` 與高程圖一起壞，height scan 錯誤會直接影響步態 | 監控 LIO 健康狀態；異常時讓策略進入保守模式或停止 |
| 電梯中 LIO 退化 | 車廂與機器人一起移動 | `odom` z 錯誤、高程圖殘留錯誤高度 | 換樓層時清除高程圖；必要時重新定 `odom` 原點 |
| TF 頻率與延遲 | LIO 10 Hz，步態策略通常 50 Hz | height scan 使用過時或預測的 base 高度 | local_ekf 提高頻率並融合高頻 IMU；評估延遲對策略的影響 |
| 步態造成的姿態抖動 | base 持續 roll / pitch | 點雲投影錯誤會在高程圖產生假地形 | 正確融合 IMU 姿態；使用 elevation_mapping 的漂移補償與變異數設定 |
| `map → odom` 平面時的不一致（第一階段） | 全域修正不含 z | RViz 中地圖與機器人高度不一致；height scan 不受影響 | 第二階段處理 |
| 控制器座標轉換 | 路徑點 z=0、base 有高度與俯仰 | 轉到 `base_link` 時有數 cm 誤差 | 一般可忽略；實測驗證 |
| 共變異數調參 | 3D 狀態量變多 | 濾波結果不一致 | 重新跑 `covariance_calibration.py` |

### 不受影響

- `base_link → sensor` 靜態 TF、`pointcloud_to_laserscan`、`collision_monitor`（皆在 `base_link` 中處理）。
- Nav2 規劃器、goal / progress checker（只使用 x / y / yaw）。
- 2D 模式（AMCL，`run_2d_localization.sh`）。

## 驗收條件

- 輪式機器人：現有驗證腳本全部通過，產生的 local_ekf / costmap 設定與目前一致。
- 四足：`odom → base_link` 的 z / roll / pitch 與 Isaac Sim ground truth 誤差在容許範圍內。
- 四足：部署端 height scan 與 Isaac Lab RayCaster 在相同場景下的誤差在容許範圍內。
- 四足：costmap 在任何樓層都能標記 0.06–2.0 m 的障礙物。
- 換樓層後高程圖已清除，不殘留電梯中的錯誤高度。
