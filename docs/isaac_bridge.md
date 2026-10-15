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

`run_slam.sh` 現在以閒置的建圖管理器啟動；使用 `./scripts/slam_session.sh start FLOOR` 才啟動轉接、FAST-LIO2 與 PGO 節點，`save-stop` 存圖成功後只停止這些節點，感測器、電梯與模擬時鐘不停止。流程及限制見[分樓層建圖管理器](mapping_flow.md#分樓層建圖管理器)。

這些轉接針對模擬；目前 launch 與預設設定不能直接套用到實機。

## 動態電梯門與升降

直接啟動模擬器時，可選擇在執行期間啟用 Office 的電梯門與車廂：

```bash
/home/yu/isaacsim-6.1.0/python.sh scripts/standalone.py --dynamic-elevator-doors
# 手動模式：初始關門、車廂在地面樓層，E 開／關門，L 升／降
/home/yu/isaacsim-6.1.0/python.sh scripts/standalone.py \
  --dynamic-elevator-doors --elevator-door-mode manual
# 無 GUI 的自動循環
/home/yu/isaacsim-6.1.0/python.sh scripts/standalone.py --headless --dynamic-elevator-doors
```

直接執行 `standalone.py` 時預設不啟用，維持原場景；`./scripts/run_slam.sh` 則自動帶上 `--dynamic-elevator-doors --elevator-door-mode manual`，以初始關門的手動模式啟動。啟用後沿用原門片的骨架、網格與材質，四片伸縮門同步向兩側滑動，另建立隱藏的運動學碰撞盒與門片同步。沿用原有車廂並以內部引用複製到另一個井口，兩個車廂都從地面樓層起始，車廂、內門與碰撞一起升降。修改僅存在 USD session layer，不儲存或改寫 NVIDIA 資產。

Office 原本的 `/Root/GroundPlane/CollisionPlane` 是 z=0 的無限碰撞平面，也會支撐看似空的電梯井。啟用動態門時，改用涵蓋原地面網格與場景地板範圍的有限三角網格碰撞地面，僅在 Office 6.1 的兩個電梯井挖出 2×2 m 開口（x=-17～-15；y=-2.5～-0.5、-5.5～-3.5），保留辦公室及門外地面的支撐；地下樓層使用原有可見地板網格建立碰撞，供車輛出入車廂。未啟用時維持原本無限地面。**空井沒有透明地板或額外井底，未停靠樓層的外門保持關閉**。

自動模式只循環停靠樓層的門，使用模擬時間：關門停留 5 秒、開門 2 秒、開門停留 5 秒、關門 2 秒；暫停模擬時也暫停動畫。GUI 按 **E** 可中途反向並切換為手動模式，即使車輛使用 `--ros-cmd-vel` 也能操作；手動模式不支援 `--headless`。

按 **L** 同步切換兩個車廂的停靠樓層：地面 `z=0` 與地下 `z=-3.3 m`。進入車廂後先停車再按 L；它會切換到手動模式，先用約 2 秒關門，再以平滑加減速用 6 秒升降。移動中 E 不開門、重複 L 不反向；抵達後門保持關閉，按 E 才開啟車廂內門與停靠樓層的外門。未停靠樓層的門不會打開，內門碰撞隨車廂移動；機器人靠物理接觸隨車廂升降，不直接改寫機器人位置。重設模擬時間會恢復地面停靠、關門狀態。

### MQTT 電梯控制（Open-RMF lift）

`--elevator-mqtt-lift NAME`（需 `--dynamic-elevator-doors --mqtt-host`，可 `--headless`）讓上位系統像控制實體電梯控制器一樣經 MQTT 控制電梯；欄位對應 Open-RMF `rmf_lift_msgs`，Open-RMF 端由 `rmf_isaacsim` 的 `mqtt_lift_adapter` 轉成 `/lift_requests`、`/lift_states`。`./scripts/run_multi_nav_online.sh --elevator-lift lift_1` 會帶上這些參數。

| Topic | 方向 | 內容 |
|-------|------|------|
| `rmf/lift/NAME/request` | 上位 → 模擬器（QoS 1） | `{"session_id":"carter1","request_type":"agv"\|"human"\|"end_session","destination_floor":"B1","door_state":"open"\|"closed"}` |
| `rmf/lift/NAME/state` | 模擬器 → 上位（retained，2 Hz 與變化時） | `lift_name`、`available_floors`、`current_floor`、`destination_floor`、`door_state`（`closed`／`moving`／`open`）、`motion_state`（`stopped`／`up`／`down`）、`current_mode`（`human`／`agv`；斷線遺言 `offline`）、`session_id` |

樓層以 `--elevator-floors`（預設 `1F:0,B1:-3.3`，名稱需與 `maps/` 樓層與 RMF level 一致）對應車廂高度。`agv` 請求取得 session，其他 session 的請求在 `end_session` 前忽略；換層時先關門、移動，抵達目的樓層停止後才依 `door_state` 開門。此模式停用自動循環。

目前兩個電梯同步控制，沒有獨立選梯或防夾偵測；請避免停在門口，升降時不要繼續駕駛車輛。未找到預期的 Office 車廂／骨架門結構時會報錯，不會默默以替代模型取代。

`run_slam.sh` 固定啟用手動電梯門，不提供模式切換參數；`run_multi_nav_online.sh` 提供 `--elevator-lift`，其他 `run_*.sh` 尚未提供電梯門參數的轉傳介面。單獨執行模擬器不會自動啟動 ROS container；要同時建圖／導航，需另行啟動相應 ROS 流程。

驗證：`python3 -m unittest tests.test_elevator_doors tests.test_mqtt_elevator` 測試控制時序、MQTT 電梯 session／開關門順序、E／L 按鍵、升降互鎖及地面開口網格；`/home/yu/isaacsim-6.1.0/python.sh tests/validate_elevator_doors.py` 載入真實 Office，檢查門片與碰撞盒一致、原始 USD 未修改、PhysX 門口阻擋、RTX LiDAR 量測，以及兩個車廂以物理接觸載送 Nova Carter 下樓再上樓、停靠樓層門互鎖與地下地板支撐。

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
