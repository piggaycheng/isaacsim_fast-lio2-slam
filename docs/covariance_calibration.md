# 更換車輛後的協方差校正

EKF 依據各感測器的 covariance 決定要相信誰。換了一台車（輪徑、輪距、LiDAR 位置、IMU 不同）後，舊的數值不再代表新車的誤差，需要重新量測。本流程只使用真車也有的資料，不需要真值。

本工具需要輪速資料（`/wheel/odom`）。`local_odometry.inputs` 不含 `wheel` 的機器人（見 [`3d_localization.md`](3d_localization.md#local-ekf-輸入選擇)）不適用；其 `lio_odometry` 的 `position_variance`／`yaw_variance` 目前為固定值，未由本工具校正。

## 會被校正的參數

全部寫在各車種 profile `ros2_ws/src/slam_localization_3d/config/robots/<type>.yaml` 的 `parameter_overrides`：

| 區段 | 參數 | 用途 |
| --- | --- | --- |
| `wheel_encoder_odometry` | `distance_variance_per_meter`、`position_variance_per_radian` | 輪速位置誤差：每走 1 m、每轉 1 rad 增加多少變異數 |
| 同上 | `yaw_variance_per_meter`、`yaw_variance_per_radian` | 輪速航向誤差：直行與原地旋轉造成的 yaw 漂移 |
| `nav_imu_adapter` | `angular_velocity_variance` | IMU 陀螺儀 z 軸角速度雜訊 |
| `global_pose_adapter` | `min_covariance_xy`、`min_covariance_yaw`、`registration_covariance_scale` | PCD（先驗地圖 ICP）定位的 x/y/yaw 誤差 |

共用的 `slam_nav/config/local_odometry.yaml` 與 `slam_localization_3d/config/global_fusion.yaml` 不含這些車種相關值；launch 載入時合併 profile，profile 缺任何一項就會在啟動時報錯，不會悄悄沿用程式預設值。

以下項目不會被自動寫入，工具只負責回報，需要手動修正：

- **LiDAR 外參（body→base）**：工具以輪子的旋轉中心推算 LiDAR 位置。若與設定相差超過約 1 cm，先核對實測／USD 安裝位置、輪子滑移與感測器時序；確認設定錯誤後，修改車種 profile 的 `sensor_frames`（`imu_link` 與 `lidar_link` 的 `x/y`）。靜態 TF、`localization_3d_pose`／`global_pose_adapter` 的 body→base 轉換與本工具的 `--lio-body-to-base` 預設值都由它推得。擬合不一定只反映安裝誤差，不應直接覆蓋已確認的實際幾何。
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
   - 速度上限：1.0 m/s、0.75 rad/s 是 Isaac Carter `CmdVelReceiver` 的限制（超過會被歸零），不是校正本身的需求。校正路線預設速度不變。真車請用與實際導航相近的速度（例如 Nav2 controller 的最高速度），不要明顯更快，以免輪子打滑、LiDAR 運動畸變與 ICP 誤差讓 covariance 偏大；也不宜過慢，否則直行與旋轉的樣本量不足。
   - 路線結束後停止錄製。
5. **計算建議值**：
   ```bash
   docker compose run --rm ros \
     python3 ros2_ws/src/slam_localization_3d/scripts/covariance_calibration.py cov_bag
   ```
   - 非 Carter 車種加上 `--robot-type <type>`（預設 `nova_carter`）。
   - 工具需要從發布的 covariance 中扣除**錄製當下**的 floor，會讀該車種 profile 的 `global_pose_adapter`；重算舊 bag 時以 `--profile-dir` 指向錄製當時的 profile 副本。
   - 在 Isaac Sim 中加上 `--ground-truth`，逐項比對真值。
6. **檢查輸出**：
   - `LIO body->base xy fitted` 與設定值相差約 1 cm 以上時，先核對幾何與時序。若確實改動外參，回到步驟 2 重錄；若實測安裝值已確認正確，工具在殘差改善未達 20% 時本來就保留設定值；需要強制保留時可加 `--keep-lever`。
   - `PCD vs LIO ... by pair lag` 的數值應隨時間間隔增加而趨於平穩。若仍持續上升，代表 LIO 漂移明顯，可縮小 `--pcd-max-lag`。
   - IMU 有兩個估計：三角比較（`hat`）假設三個來源誤差互相獨立；但陀螺儀尺度誤差與輪速 yaw 誤差都隨旋轉角度增加而相關，會讓 `hat` 低估 IMU。因此工具另以 IMU／LIO yaw 差（`IMU/LIO bound`，包含很小的 LIO 誤差，是上限）擬合，取兩者較大值，並列出 `IMU yaw scale vs LIO`；偏離 1 超過 1% 代表陀螺儀有尺度誤差。兩者都不需要真值。
   - `--min-imu-variance` 只用於有獨立 IMU 規格時設定下限，不要用真值反推的數值。驗證會使用**最終建議值**（包含靜止量測與下限）。
   - 若改了工具參數（如 `--pcd-unobservable-factor`），需再錄一份資料驗證固定參數，不能只以原本用來調整參數的資料宣稱獨立驗證通過。
7. **寫入設定**：確認沒有問題後，在同一個指令加上 `--apply`，工具只會把建議值寫入 `--robot-type` 對應 profile 的 `parameter_overrides`，不會修改共用 YAML。若使用 `--ground-truth` 且有任何項目 FAIL，工具會拒絕寫入。
8. **重新建置並重啟**：執行 `docker compose run --rm ros build --packages-select slam_nav slam_localization_3d` 重新建置，再重新啟動定位與導航。

## 輸出判讀

`--ground-truth` 的 ratio 為「真實誤差平方 ÷ 預測變異數」，1 最理想：

- `PASS`：0.5–2。
- `CONSERVATIVE`：1/3–0.5，高估但安全。
- `UPPER BOUND`：只用於 `wheel_distance` 與 `imu_yaw`。輪速前後方向的誤差只能拿 LIO 比對，無法拆成輪速與 LIO 各自的部分，所以兩者全部算在輪速上；IMU 的 IMU/LIO 上限也同樣包含 LIO 誤差。高估是預期結果，不算失敗。
- `FAIL`：低估（大於 2）或嚴重高估，會讓 EKF 過度相信或忽略該感測器。

## Carter v1／namespaced 操作

以下以 `carter2:carter_v1@3.5,0,0` 為例，所有終端使用相同 `ROS_DOMAIN_ID=192`，並先停止其他會控制這台車的定位／導航流程。

啟動 ROS 容器與**不含 Nav2**的定位（第一個終端）：

```bash
export ROS_DOMAIN_ID=192
ROS_COMMAND=idle docker compose -p carter-v1-calibration up -d ros
docker compose -p carter-v1-calibration exec -T ros /workspace/docker/entrypoint.sh \
  ros2 launch slam_localization_3d robot.launch.py \
  robot:=carter2:carter_v1@3.5,0,0 \
  map_pcd:=/workspace/maps/office/map.pcd map_pgm:=/workspace/maps/office/map_2d.yaml \
  rviz:=false auto_initial_pose:=true obstacle_cloud:=false costmaps:=false navigate:=false
```

啟動 Isaac Sim（第二個終端）：

```bash
ROS_DOMAIN_ID=192 /home/yu/isaacsim-6.1.0/python.sh scripts/standalone.py \
  --headless --ros-cmd-vel --lidar-motion-compensation noncompensated \
  --robot carter2:carter_v1@3.5,0,0
```

定位正常後錄製（第三個終端）；保留錄製時的 profile，重算舊 bag 時必須使用當時的 PCD floor：

```bash
mkdir -p ros2_ws/log/covariance/carter_v1/recording_profile
cp ros2_ws/src/slam_localization_3d/config/robots/carter_v1.yaml \
  ros2_ws/log/covariance/carter_v1/recording_profile/
docker compose -p carter-v1-calibration exec -T ros /workspace/docker/entrypoint.sh \
  ros2 bag record --use-sim-time -o ros2_ws/log/covariance/carter_v1/bag \
  /clock /carter2/wheel/odom /carter2/nav/imu /carter2/Odometry \
  /carter2/localization_3d/global_pose /carter2/isaac/ground_truth/odom
```

確認 recorder 已訂閱後，執行路線（第四個終端）。`covariance_drive.py` 發布絕對 `/cmd_vel`，**一定要 remap**：

```bash
docker compose -p carter-v1-calibration exec -T ros /workspace/docker/entrypoint.sh \
  ros2 run slam_localization_3d covariance_drive.py --ros-args \
  -p use_sim_time:=true -r /cmd_vel:=/carter2/cmd_vel
```

路線結束後以 Ctrl+C 停止 recorder，計算建議值：

```bash
docker compose -p carter-v1-calibration exec -T ros /workspace/docker/entrypoint.sh \
  python3 ros2_ws/src/slam_localization_3d/scripts/covariance_calibration.py \
  ros2_ws/log/covariance/carter_v1/bag --robot-type carter_v1 \
  --wheel-topic /carter2/wheel/odom --imu-topic /carter2/nav/imu \
  --lio-topic /carter2/Odometry --pcd-topic /carter2/localization_3d/global_pose \
  --truth-topic /carter2/isaac/ground_truth/odom --ground-truth --truth-yaw-offset 0
```

所有建議值都只由輪速／IMU／LIO／PCD 推得，工具參數皆用預設值（未用 `--keep-lever`、`--min-imu-variance` 或改 `--pcd-unobservable-factor`），模擬新車種不知道真值的情境；`--ground-truth` 只做驗證與 `--apply` 的拒寫保護，在真車上省略即可。`--truth-yaw-offset 0` 對應 Carter v1 的 +x 前進方向；Nova Carter 的預設為 π。首次分析通過後才加 `--apply`，再建置與重啟；**寫入後重算同一份 bag**，需加 `--profile-dir ros2_ws/log/covariance/carter_v1/recording_profile` 讀取原始 floor，且不要加 `--apply` 覆寫證據副本。完成後停止 Isaac Sim，並執行 `docker compose -p carter-v1-calibration down`。

## Carter v1 校正紀錄

2026-10-03 在 Office 場景以文件預設路線（4 輪，線速 0.25 m/s、原地旋轉 0.5 rad/s、弧線角速度 0.4 rad/s）錄製，每次約 362 秒。寫入的值**全部來自不使用真值的工具輸出**（預設參數）；真值只用於驗證。證據位於 `ros2_ws/log/covariance/carter_v1/`：

- `20261003_000952/`：校正用資料，以初始 profile 錄製。`analysis.log` 是舊版工具結果：只用三角比較時 IMU 真值 ratio 15.40 FAIL。診斷發現 Carter v1 的陀螺儀 yaw 比 LIO 多約 5%（相對真值亦同；Nova Carter 為 1.000），屬於隨旋轉角度增加的尺度誤差，且與輪速 yaw 誤差相關，違反三角比較的獨立假設。工具改為同時以 IMU/LIO 差估計上限後，`analysis-truth-free.log` 六項全部 PASS（IMU 0.97），`apply-truth-free.log` 為 `--apply` 寫入紀錄。
- `20261003_005528/`：**獨立驗證**。以寫入後的 profile 重新錄製，固定參數、不重新擬合（`frozen-validation.log`）；180 個窗口、168 筆 PCD pose，六項 ratio 全部 PASS，PCD RMSE 約 1.99 cm、0.080°。同一份資料重新擬合（`analysis-truth-free.log`）的建議值與寫入值一致（例如 IMU 0.0263 vs 0.0242）。
- `20261003_002217/`、`20261003_003208/`：先前以真值輔助（IMU 下限 0.024、PCD 倍率 3）的版本，已作廢，只保留作為比較；其中 `002217` 顯示預設 PCD 倍率 2 可能讓 PCD yaw ratio 達 2.39（FAIL），見限制。

| 已寫入 `carter_v1.yaml` 的參數（工具輸出） | 值 | 獨立驗證 ratio |
| --- | --- | --- |
| `distance_variance_per_meter`／`position_variance_per_radian` | `4.50555e-05`／`0.000946136` | wheel distance `0.79` |
| `yaw_variance_per_meter`／`yaw_variance_per_radian` | `0.0044976`／`0.000594103` | wheel yaw `1.34` |
| `angular_velocity_variance` | `0.0242269` | IMU yaw `0.99` |
| `min_covariance_xy` | `0.000199663` | PCD x `1.23`、y `0.70` |
| `min_covariance_yaw` | `1.24068e-06` | PCD yaw `1.52` |
| `registration_covariance_scale` | `1.0` | 包含於上列 PCD 驗證 |

輪速擬合的 body→base xy 約 `(0.047, -0.011)` m，與 USD 安裝值 `(0.060, 0)` m 有約 1.3 cm 差異（推測為 caster 摩擦造成的旋轉中心偏移），殘差改善未達 20% 門檻，工具自動保留安裝值。輪速距離尺度相對 LIO 為 0.992／0.989。IMU orientation／linear acceleration covariance 未由本流程量測，仍保留初始值。

陀螺儀 5% 尺度誤差的來源：Isaac Sim 回報的車體角速度（真值 twist 與 IMU 一致）比真值姿態的 yaw 變化率大約 5%，線速度則一致，推測與 Carter v1 caster 接觸的物理求解有關。此處把它視為新車的感測器特性，以 covariance 涵蓋，未修改模擬器；把 angular velocity covariance 加大是保守做法，若要更精準，應在 IMU 端校正尺度。

寫入後的混合車種導航結果見 [multi_robot.md](multi_robot.md#驗證)。此次沒有量測 Carter v1 在 1 m/s／0.75 rad/s 的 covariance 或煞停距離，不代表實機或所有速度／場景均已校正。

## 限制

- PCD 與 FAST-LIO 使用同一顆 LiDAR，兩者共同的誤差（例如隨車頭方向變化的掃描偏差）沒有外部參考就量不到。工具以 `--pcd-unobservable-factor`（預設 2）補償；2 是 Nova Carter 的 Isaac Sim 驗證值，Carter v1 校正 bag 與獨立驗證 bag 的 PCD yaw ratio 為 1.47／1.52，但較早一份 bag 曾達 2.39，表示此倍率在 Carter v1 上的餘量有限。這是沒有外部參考時無法從資料自行決定的先驗值，不能假設同一倍率適用所有車。若真車有 RTK 或全站儀等外部參考，應重新確認這個倍率。
- EKF 的 process noise（Q）不在此流程內。
- 在 Isaac Sim 中，`/nav/imu` 的 IMU 以 120 Hz 輸出，但時間戳只有 60 Hz，所以會出現重複時間戳。工具已去除重複；真車上應確認 IMU 時間戳是否唯一。
