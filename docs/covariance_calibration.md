# 更換車輛後的協方差校正

EKF 依據各感測器的 covariance 決定要相信誰。換了一台車（輪徑、輪距、LiDAR 位置、IMU 不同）後，舊的數值不再代表新車的誤差，需要重新量測。本流程只使用真車也有的資料，不需要真值。

## 會被校正的參數

| 設定檔 | 參數 | 用途 |
| --- | --- | --- |
| `ros2_ws/src/slam_nav/config/local_odometry.yaml` → `wheel_encoder_odometry` | `distance_variance_per_meter`、`position_variance_per_radian` | 輪速位置誤差：每走 1 m、每轉 1 rad 增加多少變異數 |
| 同上 | `yaw_variance_per_meter`、`yaw_variance_per_radian` | 輪速航向誤差：直行與原地旋轉造成的 yaw 漂移 |
| 同上 → `nav_imu_adapter` | `angular_velocity_variance` | IMU 陀螺儀 z 軸角速度雜訊 |
| `ros2_ws/src/slam_localization_3d/config/global_fusion.yaml` → `global_pose_adapter` | `min_covariance_xy`、`min_covariance_yaw`、`registration_covariance_scale` | PCD（先驗地圖 ICP）定位的 x/y/yaw 誤差 |

多車種時，這些值實際寫在各車種 profile（`config/robots/<type>.yaml`）的 `parameter_overrides`；Nova Carter 的值同時存在 profile 與上述共用預設 YAML。

以下項目不會被自動寫入，工具只負責回報，需要手動修正：

- **LiDAR 外參（body→base）**：工具以輪子的旋轉中心推算 LiDAR 實際位置。若與設定相差超過約 1 cm，修改車種 profile 的 `sensor_frames`（`imu_link` 與 `lidar_link` 的 `x/y`）。靜態 TF、`localization_3d_pose`／`global_pose_adapter` 的 body→base 轉換與本工具的 `--lio-body-to-base` 預設值都由它推得。
- **輪子幾何**：工具會回報 `wheel systematic yaw drift` 與 `distance scale vs LIO`。若明顯偏離 0 與 1，代表輪徑或輪距設定錯誤，應先修正幾何，再校正 covariance。

輪速 covariance 是**動態**的：節點依照每一步實際移動的距離與轉角累積誤差，靜止時不再增加。PCD covariance 則是 ICP 本身的 covariance 加上下限（floor）。ICP 值通常只有毫米等級，所以實際上是 floor 在決定大小。

## 步驟

1. **先確認幾何設定**：輪徑、輪距、LiDAR 外參先填入新車的量測值，並重新建置。
2. **啟動定位（不啟動 Nav2）**：
   - Isaac Sim：`./scripts/run_3d_localization.sh --global-fusion --ros-cmd-vel`
   - 真車：啟動相同的定位節點，並確保 `/cmd_vel` 可以驅動底盤，周圍需要至少約 1.5 m 的淨空。
3. **錄製 rosbag**：Isaac Sim 模式下，ROS 節點跑在 `ros` 容器內，以下 ROS 指令都透過 `docker compose exec` 在容器中執行。容器的工作目錄 `/workspace` 就是專案目錄，bag 會存到專案內。
   ```bash
   docker compose exec ros /workspace/docker/entrypoint.sh \
     ros2 bag record -o cov_bag /wheel/odom /nav/imu /Odometry /localization_3d/global_pose
   ```
   在 Isaac Sim 中可以另外加上 `/isaac/ground_truth/odom`，用於驗證。
4. **執行校正路線**：
   ```bash
   docker compose exec ros /workspace/docker/entrypoint.sh \
     ros2 run slam_localization_3d covariance_drive.py --ros-args -p use_sim_time:=true
   ```
   - 內容依序為：靜止 40 秒；前進後退、正反原地旋轉、左右弧線並原路退回，重複 4 輪；最後靜止 20 秒。全程約 6 分鐘，活動範圍約 1 m。
   - 路線的目的：讓車在靜止、直行、原地旋轉、弧線等不同運動下各累積足夠樣本。靜止段量 IMU 偏差與 PCD 抖動；直行量每公尺誤差；旋轉量每弧度誤差並推算 LiDAR 外參。每個動作都原路退回，所以只需要很小的空間。
   - 真車注意事項：
     - 腳本是開環控制，直接發布 `/cmd_vel`，不經過 Nav2 或碰撞檢查。請清出至少 1.5 m 半徑的空地，並由人員拿著急停。
     - 必須設定 `use_sim_time:=false`，否則腳本會等待模擬時間而不動作。
     - 輪子打滑或底盤跟不上指令時，請降低 `linear_speed`、`spin_speed`、`arc_angular_speed`，例如 `-p linear_speed:=0.2 -p spin_speed:=0.4`。
     - 需在 PCD 地圖涵蓋、定位正常的區域內執行。
   - 也可以用搖桿手動駕駛取代腳本，工具不依賴固定路線。只要 bag 包含開頭靜止 ≥40 秒、結尾靜止約 20 秒，以及多次直行、正反原地旋轉與左右轉彎即可。
   - 速度上限：0.75 m/s、0.7 rad/s 是 Isaac Carter `CmdVelReceiver` 的限制（超過會被歸零），不是校正本身的需求。真車請用與實際導航相近的速度（例如 Nav2 controller 的最高速度），不要明顯更快，以免輪子打滑、LiDAR 運動畸變與 ICP 誤差讓 covariance 偏大；也不宜過慢，否則直行與旋轉的樣本量不足。
   - 路線結束後停止錄製。
5. **計算建議值**：
   ```bash
   docker compose run --rm ros \
     python3 ros2_ws/src/slam_localization_3d/scripts/covariance_calibration.py cov_bag \
     --local-config ros2_ws/src/slam_nav/config/local_odometry.yaml \
     --fusion-config ros2_ws/src/slam_localization_3d/config/global_fusion.yaml
   ```
   - 非 Carter 車種加上 `--robot-type <type>`（預設 `nova_carter`）。
   - 工具需要從發布的 covariance 中扣除**錄製當下**的 floor：先讀該車種 profile 的 `global_pose_adapter`，沒有才讀 `--fusion-config`。
   - 在 Isaac Sim 中加上 `--ground-truth`，逐項比對真值。
6. **檢查輸出**：
   - `LIO body->base xy fitted` 與設定值相差約 1 cm 以上時，先修正外參，再回到步驟 2 重錄。
   - `PCD vs LIO ... by pair lag` 的數值應隨時間間隔增加而趨於平穩。若仍持續上升，代表 LIO 漂移明顯，可縮小 `--pcd-max-lag`。
7. **寫入設定**：確認沒有問題後，在同一個指令加上 `--apply`，工具會把建議值寫入 `--robot-type` 對應 profile 的 `parameter_overrides`；`nova_carter` 另同步寫回上述兩個共用 YAML。若使用 `--ground-truth` 且有任何項目 FAIL，工具會拒絕寫入。
8. **重新建置並重啟**：執行 `docker compose run --rm ros build --packages-select slam_nav slam_localization_3d` 重新建置，再重新啟動定位與導航。

## 輸出判讀

`--ground-truth` 的 ratio 為「真實誤差平方 ÷ 預測變異數」，1 最理想：

- `PASS`：0.5–2。
- `CONSERVATIVE`：1/3–0.5，高估但安全。
- `UPPER BOUND`：只用於 `wheel_distance`。輪速前後方向的誤差只能拿 LIO 比對，無法拆成輪速與 LIO 各自的部分，所以兩者全部算在輪速上。高估是預期結果，不算失敗。
- `FAIL`：低估（大於 2）或嚴重高估，會讓 EKF 過度相信或忽略該感測器。

## 限制

- PCD 與 FAST-LIO 使用同一顆 LiDAR，兩者共同的誤差（例如隨車頭方向變化的掃描偏差）沒有外部參考就量不到。工具以 `--pcd-unobservable-factor`（預設 2）補償，這個倍率是在 Isaac Sim 以真值驗證得到的。若真車有 RTK 或全站儀等外部參考，應重新確認這個倍率。
- EKF 的 process noise（Q）不在此流程內。
- 在 Isaac Sim 中，`/nav/imu` 的 IMU 以 120 Hz 輸出，但時間戳只有 60 Hz，所以會出現重複時間戳。工具已去除重複；真車上應確認 IMU 時間戳是否唯一。
