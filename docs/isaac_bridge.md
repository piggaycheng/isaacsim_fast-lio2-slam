# Isaac Sim 橋接與 Docker 運作

Isaac Sim（`scripts/standalone.py`）在主機執行，ROS 2 Humble 節點（FAST-LIO2、PGO、定位、Nav2、RViz）在 `ros` Docker Compose service 內執行，主機不需安裝 ROS 2。環境建置見根目錄 [README](../README.md)。

## Docker 運作方式

- `docker/Dockerfile` 安裝所有 apt／pip 依賴（GTSAM、Nav2、`robot_localization`、`pointcloud_to_laserscan`、`pcl_ros`、Open3D…），並由 submodule 建置 Livox-SDK2 與 Sophus。
- 專案目錄掛載到 `/workspace`，修改 `ros2_ws/src` 只需重新 `docker compose run --rm ros build`（額外參數會傳給 `colcon build`），只有依賴變更才需重建 image。image 使用者預設 UID/GID 1000，不同時以 `HOST_UID=$(id -u) HOST_GID=$(id -g) docker compose build` 建置。
- 每個 `run_*.sh` 先驗證參數，以 `docker compose up` 啟動 `ros` service 並追蹤 log，再在主機啟動 `scripts/standalone.py`；Isaac Sim 結束或按 Ctrl+C 時停止 container。
- container 使用 host network 與 IPC，DDS（含共享記憶體）可直接連到 Isaac Sim 的 ROS 2 bridge；`ROS_DOMAIN_ID`、`ROS_LOCALHOST_ONLY`、`RMW_IMPLEMENTATION` 從目前 shell 傳入。地圖、PCD 與輸出路徑必須在專案目錄內，因為只有它被掛載。
- 在 container 內執行其他 ROS 指令：

```bash
docker compose run --rm ros ros2 topic list                               # 獨立 container
docker compose exec ros /workspace/docker/entrypoint.sh ros2 topic list   # run 腳本執行中
```

- **tf2 修補**：apt 的 `ros-humble-tf2` 低於 0.25.24 時，Dockerfile 會建置上游 tf2 0.25.24 並只替換 image 內的 `libtf2.so`。舊版 tf2 在 TF listener 與 costmap message filter 之間有 lock-order 死鎖（ros2/geometry2#990），數分鐘後 Nav2 伺服器的 TF buffer 會凍結，出現 `Transform data too old` 與旋轉的 local costmap。apt 版本含修正後，重建 image 會自動略過替換。

## Isaac Sim → FAST-LIO 資料

`scripts/standalone.py` 發布：

- `/isaac/lidar_points`（`sensor_msgs/msg/PointCloud2`）
- `/isaac/imu`（`sensor_msgs/msg/Imu`）
- `/clock`（`rosgraph_msgs/msg/Clock`）

並由 submodule 提供 FASTLIO2_ROS2、`livox_ros_driver2`、Livox-SDK2 與 Sophus 1.22.10。

- **點雲轉換**：FASTLIO2_ROS2 需要 `/livox/lidar`（`livox_ros_driver2/msg/CustomMsg`）。`isaac_fastlio_adapter` 轉換 Isaac 點雲並補上 `line`、`tag`、`offset_time`、反射率。RTX 點雲原始只含 XYZ 與少數欄位，這些 Livox 欄位在模擬中是近似值。
- **時間戳與去畸變**：啟用 Motion BVH；LiDAR publisher 輸出原生逐點時間戳、intensity、emitter ID 與 channel ID，原始輸出設為 `NONCOMPENSATED`，保留運動畸變讓 FAST-LIO 以模擬 IMU 去除。adapter 直接使用原生時間戳做 deskew，不再人為在掃描內均分，並把 XT-32 的 channel ID 折疊成 FAST-LIO 接受的四個 Livox line ID（保留點的順序與時間）。
- **IMU**：`imu_scale_adapter` 補償上游 FASTLIO2 把標準 IMU 加速度乘以 10 的行為，輸出到 `/livox/imu`。IMU 與 RTX LiDAR 在 `standalone.py` 中同位置，所以 `isaac_lio.yaml` 使用單位矩陣外參。

這些轉接針對模擬；目前 launch 與預設設定不能直接套用到實機。

## 套件分工

| 套件 | 內容 |
| :-- | :-- |
| `isaac_fastlio_adapter` | Isaac Sim 感測器轉接（建圖輸入） |
| `slam_nav` | 輪速里程計、導航 IMU adapter、地面障礙濾除、Nav2 `GoalHeadingLatchedRPP` 與 `LatchedGoalChecker`、共用 Local EKF／scan 設定（`config/local_odometry.yaml`） |
| `slam_localization_2d` | AMCL 設定、2D 地圖與 AMCL launch、RViz 設定 |
| `slam_localization_3d` | 3D 定位 launch、pose／TF 發布、融合、Nav2 設定與車種 profile（`config/robots/`） |
| `slam_bringup` | 整合啟動（不含節點）：`robot.launch.py robot:=NAME[:TYPE][#FLEET]@X,Y[,YAW]` 啟動單車的定位／導航（`slam_localization_3d/global_fusion.launch.py`）、profile 有雲台時的 `slam_sensor_control/gimbals.launch.py`，以及給了 `mqtt_host` 時的 `slam_fleet_bridge`；`ceiling_cameras.launch.py`（`count`，預設 4；`tilt_limits_deg`；`mqtt_*`）啟動天花板相機的雲台控制。MQTT 參數只在這一層，定位套件不接收 |
| `slam_sensor_control` | 控制車上感測器的 action。目前有 `gimbal_action_server.py`：action `gimbal/move`（`slam_sensor_control/action/GimbalMove`，目標 `pan`／`tilt`，rad）發布目標到 `gimbal/joint_command`，並監聽 `gimbal/joint_states`，姿態在容差內（參數 `pan_tolerance_deg`／`tilt_tolerance_deg`，預設 1°）才回傳 success；`timeout`（預設 30 s）內未到達或沒有雲台狀態則 abort，新目標會取代舊目標，tilt 超出 `tilt_limits_deg` 的目標會被拒絕。另有相對轉動 action `gimbal/rotate`（`GimbalRotate`，`delta_pan`／`delta_tilt`，rad）：以目標開始時的目前姿態加上增量，`|delta_pan|` 須 ≤ π（否則拒絕），算出的 tilt 超出限位則 abort，其餘行為同 `gimbal/move`，例如 `ros2 action send_goal /NAME/gimbal/rotate slam_sensor_control/action/GimbalRotate "{delta_pan: 0.5, delta_tilt: 0.0}"`。在車的 namespace 下執行：`ros2 run slam_sensor_control gimbal_action_server.py --ros-args -r __ns:=/NAME`，測試：`ros2 action send_goal /NAME/gimbal/move slam_sensor_control/action/GimbalMove "{pan: 1.57, tilt: 0.2}"`。**MQTT 前端** `gimbal_mqtt_bridge.py`（同 namespace 下與 action server 一起執行，`--ros-args -r __ns:=/NAME -p mqtt_host:=HOST`，另有 `mqtt_port`／`mqtt_username`／`mqtt_password`／`mqtt_topic`（預設 `slam/gimbal`）／`state_rate`）讓外部系統以**角度（度）**控制雲台：發布 JSON 到 `slam/gimbal/NAME/command`——`{"cmd_id":1,"action":"move","pan_deg":90,"tilt_deg":10}`（絕對，兩者必填）、`{"cmd_id":2,"action":"rotate","pan_deg":15,"tilt_deg":0}`（相對，缺省為 0）、`{"cmd_id":3,"action":"cancel"}`；`cmd_id` 為整數或字串。結果發布在 `slam/gimbal/NAME/result`（`cmd_id`、`status`：`succeeded`／`aborted`／`canceled`／`rejected`、`message`、最終 `pan_deg`／`tilt_deg`）；實測姿態以 `{pan_deg, tilt_deg, busy, current_cmd_id}` 約 2 Hz 發布在 `slam/gimbal/NAME/state`；`slam/gimbal/NAME/status` 為 retained `{"online":true/false}`（異常斷線由 LWT 發布 false）。新指令會取代進行中的舊指令（舊指令回報 `aborted`），格式錯誤的指令回報 `rejected`。需安裝 `paho-mqtt`。`launch/gimbals.launch.py` 一次啟動多組：`namespaces:="carter1;ceiling_cams/ceiling_cam_1"`（`;` 分隔，可多段）、`tilt_limits_deg`（預設 `-90,90`，須與模擬端限位一致）、`mqtt_host`（空則只啟動 action server）／`mqtt_port`／`mqtt_username`／`mqtt_password`／`mqtt_topic`。`slam_bringup/robot.launch.py` 對 profile 有 `simulation.gimbal` 的車自動 include 它（MQTT 參數沿用該 launch 的參數）；`run_multi_nav_online.sh` 另以 `slam_bringup/ceiling_cameras.launch.py` 為四台天花板相機啟動 `ceiling-gimbals` 容器（namespace `ceiling_cams/ceiling_cam_N`，MQTT topic `slam/gimbal/ceiling_cams/ceiling_cam_N/*`） |
| `FAST_LIO_LOCALIZATION2`（submodule） | 上游 3D 定位節點（固定版本） |

車種相關的輪速與 covariance 值不在 `local_odometry.yaml`，launch 會合併車種 profile，見 [多車文件](multi_robot.md#新增車種)。
