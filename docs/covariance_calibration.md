# EKF covariance 校正操作

本流程使用 `covariance_calibration.py`，從輪速、IMU、FAST-LIO 與 PCD 定位資料估算量測 covariance，再寫入車種設定檔。**估算不需要真值；Isaac Sim 真值只用於驗證。**

## 校正範圍

結果寫在 `ros2_ws/src/slam_localization_3d/config/robots/<type>.yaml` 的 `parameter_overrides`：

| 區段 | 會寫入的參數 | EKF 用途 |
| --- | --- | --- |
| `wheel_encoder_odometry` | `distance_variance_per_meter`、`position_variance_per_radian`、`yaw_variance_per_meter`、`yaw_variance_per_radian` | 輪速位置與 yaw 的累積誤差 |
| `nav_imu_adapter` | `angular_velocity_variance` | IMU yaw 角速度 |
| `lio_odometry` | `position_variance`、`yaw_variance` | local EKF 的 `/lio/odom` 每次掃描位姿雜訊，不含長時間漂移 |
| `global_pose_adapter` | `min_covariance_xy`、`min_covariance_yaw`、`registration_covariance_scale` | global EKF 的 PCD 定位誤差 |

**不會校正**：EKF process noise（Q）、IMU orientation／linear acceleration covariance，以及 `global_fusion.yaml` 中 `/lio/twist` 的 `twist_linear_variance`／`twist_angular_variance`。Global 的 LIO 參數與 local 分開，不會被此流程修改。

**必要條件**：必須有輪速、IMU、FAST-LIO 與有效的 PCD 定位資料。即使之後 local EKF 只用 `lio`，校正時仍需輪速作為參考；目前工具不支援完全沒有輪速計的機器人。

## 操作步驟

以下指令都從專案根目錄執行，以 **Nova Carter、無 namespace、Isaac Sim** 為例。其他車種與真車的差異見文末。

### 1. 確認車種設定並建置

先確認 `nova_carter.yaml` 的輪徑、輪距、encoder 方向與 `sensor_frames` 安裝位置正確。校正 covariance 不能代替幾何或感測器尺度校正。

```bash
export HOST_UID=$(id -u) HOST_GID=$(id -g)
docker compose run --rm ros build --packages-select slam_nav slam_localization_3d
```

### 2. 啟動定位

終端 A：

```bash
./scripts/run_3d_localization.sh --global-fusion --ros-cmd-vel \
  --local-inputs wheel,imu
```

不要加 `--navigate` 或同時執行其他控制 `/cmd_vel` 的程式。等待 PCD 定位正常，確認終端 B 的指令列出以下四個 topic：

```bash
docker compose exec ros /workspace/docker/entrypoint.sh ros2 topic list --no-daemon
```

必須有 `/wheel/odom`、`/nav/imu`、`/Odometry`、`/localization_3d/global_pose`；**LIO 要錄原始 `/Odometry`，不是 `/lio/odom`。**

### 3. 備份設定並開始錄製

終端 B：保留錄製當下的 profile；工具分析 PCD covariance 時需要當時的 floor。每次校正使用新的資料夾名稱，避免覆蓋舊 bag。

```bash
mkdir -p ros2_ws/log/covariance/run1/recording_profile
cp ros2_ws/src/slam_localization_3d/config/robots/nova_carter.yaml \
  ros2_ws/log/covariance/run1/recording_profile/

docker compose exec ros /workspace/docker/entrypoint.sh \
  ros2 bag record --use-sim-time -o ros2_ws/log/covariance/run1/bag \
  /clock /wheel/odom /nav/imu /Odometry /localization_3d/global_pose \
  /isaac/ground_truth/odom
```

等 recorder 顯示已訂閱上述 topic，再執行下一步。

### 4. 執行校正路線

終端 C：

```bash
docker compose exec ros /workspace/docker/entrypoint.sh \
  ros2 run slam_localization_3d covariance_drive.py --ros-args \
  -p use_sim_time:=true
```

路線約 6 分鐘：靜止 40 秒，前進後退、正反原地旋轉與左右弧線共 4 輪，最後靜止 20 秒。看到 `Calibration drive finished` 後，在終端 B 按 **Ctrl+C** 停止錄製。

**安全注意**：腳本直接發布 `/cmd_vel`，不經過 Nav2 或碰撞檢查。需至少約 1.5 m 半徑淨空；真車必須有人持急停。出現打滑或跟不上指令時先降速再重錄。

### 5. 計算並檢查建議值

```bash
docker compose exec ros /workspace/docker/entrypoint.sh \
  python3 ros2_ws/src/slam_localization_3d/scripts/covariance_calibration.py \
  ros2_ws/log/covariance/run1/bag --robot-type nova_carter --ground-truth
```

寫入前確認：

| 輸出 | 處理方式 |
| --- | --- |
| 靜止段、運動視窗及 `Recommended parameters` | 不應出現缺少資料警告；應包含上表四個區段 |
| `LIO body->base xy fitted` | 與設定差約 1 cm 以上時，先檢查安裝位置、打滑與時序，不要直接套用擬合外參 |
| `distance scale vs LIO`、`IMU yaw scale vs LIO` | 距離尺度應接近 1；IMU 尺度偏離 1 超過 1% 時先檢查尺度誤差 |
| `Ground-truth validation` | `PASS`、`CONSERVATIVE`、`UPPER BOUND` 可接受；任何 `FAIL` 都先查原因、修正並重錄 |

真值 ratio 是「實際誤差平方 ÷ 預測變異數」，接近 1 最理想。`lio over 3/10 scans` 是漂移參考，不參與通過判定。不要用真值反推下限或倍率，只為了讓檢查通過。

### 6. 寫入車種 profile

**錄製後到寫入前不要改動 profile。** 確認上一步正常，再以相同指令加 `--apply`：

```bash
docker compose exec ros /workspace/docker/entrypoint.sh \
  python3 ros2_ws/src/slam_localization_3d/scripts/covariance_calibration.py \
  ros2_ws/log/covariance/run1/bag --robot-type nova_carter --ground-truth --apply

git diff -- ros2_ws/src/slam_localization_3d/config/robots/nova_carter.yaml
```

`--apply` 會寫入所有有估算結果的區段，**不是只更新 LIO**。若只要更新其中一組，請將 `Recommended parameters` 對應值手動填入 profile，其他值不動。使用 `--ground-truth` 時，任何 `FAIL` 都會阻止寫入。

寫入後若要重算同一份 bag，加入 `--profile-dir ros2_ws/log/covariance/run1/recording_profile`，並且**不要加 `--apply`**，否則會修改備份而非正式 profile。

### 7. 重新建置、重啟並確認生效

在終端 A 按 Ctrl+C 停止定位與模擬，再執行：

```bash
docker compose down
docker compose run --rm ros build --packages-select slam_nav slam_localization_3d
```

重新以實際要用的 local EKF inputs 啟動，例如 `--local-inputs lio,imu`。用以下方式確認節點讀到新值（其餘參數依校正範圍表替換）：

```bash
docker compose exec ros /workspace/docker/entrypoint.sh \
  ros2 param get --no-daemon /lio_odometry position_variance
```

再跑一次路線，確認定位、TF 連續性與實際誤差沒有退化。若有外部真值，應用新的資料驗證；重新擬合新 bag 得到 PASS，不等於已驗證先前寫入的固定參數。

## 其他車種、namespace 與真車

| 情境 | 必須調整的地方 |
| --- | --- |
| 其他車種 | 啟動對應車種，備份其 `<type>.yaml`，分析／寫入都加 `--robot-type <type>` |
| 有 namespace，例如 `carter2` | 錄製 topic 加 `/carter2` 前綴；分析加 `--wheel-topic /carter2/wheel/odom --imu-topic /carter2/nav/imu --lio-topic /carter2/Odometry --pcd-topic /carter2/localization_3d/global_pose --truth-topic /carter2/isaac/ground_truth/odom`；路線指令加 `-r /cmd_vel:=/carter2/cmd_vel`；查參數用 `/carter2/<node>` |
| 真車 | 啟動相同感測器與定位節點；錄製省略 `--use-sim-time`、`/clock`、真值 topic；路線設 `-p use_sim_time:=false`；分析／寫入省略 `--ground-truth` |
| 重算舊 bag | `--profile-dir` 指向錄製時的 profile 備份；只分析，不加 `--apply` |

**真車的 `--ground-truth` 註記**：目前此選項預期讀取 Isaac Sim 的真值 odometry；真車沒有這份資料時應省略。不加此選項仍可估算 covariance 並用 `--apply` 寫入，但不會做真值比對，也沒有真值驗證 FAIL 的拒寫保護。若實車有 RTK、全站儀等外部參考，需先處理座標系、車體參考點與時間同步，不能直接當成模擬真值套用。

PCD 與 FAST-LIO 共用 LiDAR，部分共同誤差無法靠兩者比較量到；工具的 `--pcd-unobservable-factor` 預設 2，不保證適用所有車輛與場景。有 RTK、全站儀等外部參考時，應另外確認 PCD covariance 是否合理。
