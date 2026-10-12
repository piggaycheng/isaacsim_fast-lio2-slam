# 對外連線（MQTT／RTSP）

`./scripts/run_multi_nav_online.sh` 與 `run_multi_nav.sh` 的導航、多車參數相同（見[多車文件](multi_robot.md)），另外預設啟動會對外的連線：MQTT client（相機列表與開關、車端 Open-RMF 介面）與 RTSP 相機串流。`run_multi_nav.sh` 不含這些功能，也不需要 broker。

```bash
./scripts/run_multi_nav_online.sh --help
```

Isaac Sim 預設以 headless 執行（無 GUI），要顯示 GUI 請加 `--gui`。

## 相機影像（RTSP）

相機以 RTSP（H.264）串流，**不發 ROS image topic**，開關也只走 **MQTT**，因此只有 `./scripts/run_multi_nav_online.sh` 提供相機（`run_multi_nav.sh` 不含相機與 MQTT）。所有相機預設關閉：收到 `true` 才建立 render product 並開始串流，收到 `false` 即停止並釋放，關閉時不耗算圖資源。

```bash
./scripts/run_multi_nav_online.sh --robot carter1@0,0 \
  --camera carter1=chassis_link/sensors/front_owl/camera

# 開啟／關閉（payload 為 true 或 false，不要 retain）
mosquitto_pub -h localhost -t slam/cameras/ceiling_cams/ceiling_cam_1/enable -m true
# 觀看
ffplay rtsp://HOST:8554/ceiling_cams/ceiling_cam_1
```

- **天花板雲台相機**（預設提供，`--no-ceiling-cameras` 關閉）：`ceiling_cam_1`～`4` 位於 Office 座標 (−14, 10)、(4.5, 10)、(4.5, −10)、(−14, −10)，高 2.9 m，初始朝 (0, 0, 0) 地板。開關 topic `slam/cameras/ceiling_cams/ceiling_cam_N/enable`，串流 `rtsp://HOST:8554/ceiling_cams/ceiling_cam_N`。位置與高度在 `standalone.py` 的 `CEILING_CAMERA_*` 常數。每台都有運動學 pan/tilt 雲台（純 transform，無物理；prim `/World/CeilingCameras/ceiling_cam_N/gimbal_pan/gimbal_tilt/camera`）：pan 0 = Office +x、逆時針為正，tilt 正值向下、限位 ±90°，限速 60°/s（`CEILING_GIMBAL_*` 常數）。ROS 介面與車載雲台相同，namespace 為 `/ceiling_cams/ceiling_cam_N`（`gimbal/joint_command`／`gimbal/joint_states`）；`run_multi_nav_online.sh` 會啟動 `ceiling-gimbals` 容器（`slam_bringup/ceiling_cameras.launch.py`，內部 include `slam_sensor_control/gimbals.launch.py`）執行四組 action server 與 MQTT bridge，外部以 `slam/gimbal/ceiling_cams/ceiling_cam_N/command` 控制（協定見 [isaac_bridge.md](isaac_bridge.md) 的 `slam_sensor_control`）。
- 有雲台的相機在相機列表的項目會多一個 `gimbal_topic`（該相機雲台的 MQTT command topic），前綴由 `--gimbal-mqtt-topic`（預設 `slam/gimbal`）決定，須與 bridge 的 `mqtt_topic` 一致。
- **車載相機**：`--camera [ROBOT=]CAMERA`（可重複）。`CAMERA` 是相對於車輛 prim 的相機路徑或絕對 prim 路徑；`NAME` 為路徑去掉 `chassis_link/sensors/` 後以 `_` 連接，如 `front_owl_camera`。開關 topic `slam/cameras/[ROBOT/]camera/NAME/enable`，串流 `rtsp://HOST:8554/[ROBOT/]camera/NAME`。設有 `simulation.gimbal` 的車（見 [multi_robot.md](multi_robot.md)）在指定 `--mqtt-host` 時，雲台相機自動加入列表，名稱 `gimbal`（`camera/gimbal`），同樣預設關閉、收到 enable 才串流，不需 `--camera`；雲台姿態控制見 multi_robot.md。Nova Carter 的相機：`front_owl`／`left_owl`／`right_owl`／`back_owl`（`.../camera`）及 `front_hawk` 等（`.../left/camera_left`）。
- **MQTT**：`--mqtt-host`（預設 `$MQTT_HOST` 或 `localhost`）、`--mqtt-port`（1883）、`--mqtt-topic`（`slam/cameras`）、`--mqtt-username/--mqtt-password`。相機列表以 retained JSON 發布在 `slam/cameras`（`name`（識別用，同 `rtsp_path`）、`display_name`（給介面顯示，如 `Ceiling camera 1`、`a gimbal`）、`rtsp_path`、`enable` topic、解析度、fps、`enabled`），僅在相機開關變動時更新，結束時發布 `{"online":false}`（異常斷線由 LWT 發布）。Broker 可晚於模擬啟動，client 會在背景重連。需在 Isaac Sim Python 安裝 `paho-mqtt`。
- `--camera-resolution WxH`（預設 640x480）、`--camera-fps N`（預設 15，最高 60）。
- RTSP server 是 MediaMTX（Docker image `bluenviron/mediamtx:latest-ffmpeg`，host network，埠 8554），第一次開啟相機時才啟動；編碼用同一 image 內的 ffmpeg（`libx264`）。首次使用會自動下載 image。瀏覽器不能直接播 RTSP，網頁需另行轉成 WebRTC／HLS（MediaMTX 也提供，埠 8889／8888，但此專案未設定）。

## 車端 MQTT 介面（Open-RMF fleet adapter）

`run_multi_nav_online.sh` 會在每台車的容器內啟動 `slam_fleet_bridge` 套件的 `fleet_bridge_node.py`（namespace `/NAME`）。每台車只用**一條** MQTT 連線，依 Open-RMF 介面規格（`open_rmf/docs/amr_mqtt_interface_spec.md`）的 `rmf/F/robot/NAME/*` topic 與 fleet adapter 溝通，adapter 不必使用 ROS。協定編解碼在 `rmf_protocol.py`，任務由 py_trees 執行（`task_bt.py`）。

```bash
./scripts/run_multi_nav_online.sh --robot carter1#tinyRobot@0,0 --robot carter2:carter_v1#otherFleet@3.5,0
```

`F` 為該車的車隊名稱，寫在 `--robot` 規格的 `#FLEET`（`NAME[:TYPE][#FLEET]@X,Y[,YAW]`，在座標之前），省略為 `default_fleet`；不指定 `--robot` 時預設 `carter1` 在 `fleet1`、`carter2` 在 `fleet2`。不同車隊的車各自使用自己的 `rmf/F/...` topic。broker 使用與相機相同的 `--mqtt-host/--mqtt-port/--mqtt-username/--mqtt-password`。單獨使用：`ros2 launch slam_bringup robot.launch.py robot:=NAME#F@X,Y ... mqtt_host:=HOST`（`mqtt_host` 為空時不啟動）。

| Topic（`rmf/F/robot/NAME/`） | 方向 | 內容 |
| :-- | :-- | :-- |
| `register` | 車 → adapter（QoS 1） | 初始位置與車體規格 |
| `register_ack` | adapter → 車（QoS 1） | 註冊結果 |
| `heartbeat` | 車 → adapter（QoS 0） | `x`、`y`、`yaw`、`battery`、`status`、`current_cmd_id` |
| `command` | adapter → 車（QoS 1） | `navigate`、`dock`、`stop`、`task`（擴充） |
| `command_result` | 車 → adapter（QoS 1） | `completed`／`failed`／`canceled` |
| `deregister` | 車 → adapter（QoS 1） | 正常關機 |
| `status` | broker → adapter（LWT） | 異常斷線時的 `offline` |
| `task_state` | 車 → adapter（QoS 1，retained） | 擴充：任務進度 |

- Client ID `amr_F_NAME`，clean session，keepalive 20 s。
- 流程：連線並取得 `odometry/global` 後送 `register`，未收到成功的 `register_ack` 每 3 秒重送（fleet adapter 比車晚啟動也能註冊上）；MQTT 重連不會重新註冊（`register` 代表新 session，adapter 會中斷進行中的任務），只在 adapter 回 `require_register` 時才重新註冊。收到 `status: success` 後才接受 `command`，之前收到的指令會被忽略。結束時送 `deregister`。
- 晚啟動／重啟自癒：註冊完成前也持續送 `heartbeat`（`status` 為 `error`，不是 `idle`），adapter 發現未知車輛會回 `register_ack` 的 `status: require_register`，車端收到後立即重送 `register` 並回到未註冊狀態（執行中的任務不中斷）。
- `heartbeat`（預設 2 Hz）：`x`、`y`、`yaw`（`map` 座標，rad）、`battery`、`status`（已註冊：`moving` 任務執行中或取消中，否則 `idle`；未註冊：`error`）、`current_cmd_id`（無任務為 `null`；取消中仍為舊任務）。位置與電量為 node 屬性，`status` 與 `current_cmd_id` 每次從任務執行器現算。
- `command`：`navigate` 與 `dock` 都轉成一個 Nav2 目標（`target.x/y/yaw`；`dock` 不做額外對位或充電動作，`speed_limit` 目前不套用）。新的 `cmd_id` 會非同步取消執行中的任務，等 Nav2 回傳舊導航的最終結果後才回報舊任務結果並開始新任務；即使取消發生在等待 Nav2 接受目標時，也會在取得 goal handle 後送出取消。取消期間不執行後續步驟，若收到更多新任務只保留最新一份，被取代的待執行任務回 `canceled`。執行中或待執行的相同 `cmd_id` 重送會被忽略；已結束的指令沒有去重紀錄。`stop` 也會清除待執行任務，確認舊導航結束後才回 `completed`，不是確認車已完全停穩。`robot_id` 與本車不同的指令會被忽略，格式錯誤的指令回 `failed`。
- 取消確認：接受取消請求不等於導航已結束，必須等舊 goal 的最終結果。Nav2 回 `canceled` 才回報取消；若最後一步已成功則仍回 `completed`，導航失敗回 `failed`；若僅中間步驟成功，因剩餘步驟已停止，整體回 `canceled`。取消被拒絕、非同步請求失敗或超過 `cancel_timeout`（node 參數，預設 10 秒）仍未結束時，記錄錯誤，待執行的新任務及等待中的 `stop` 回 `failed`；舊任務保持 `canceling` 並持續追蹤最終結果，確認結束前不接受新導航，不假裝取消成功。
- `command_result`：`completed`（到位）、`failed`（導航失敗或指令錯誤）、`canceled`（被新指令或 `stop` 中斷），並附 `final_location`。
- `battery`：沒有電池模型，預設固定 `100.0`；有 `sensor_msgs/BatteryState` 發布到 `/NAME/battery_state` 時改用其 `percentage`。
- 可調參數（`fleet_bridge` node 參數）：`level_name`（`L1`）、`waypoint_name`、`default_charger`、`default_parking`、`footprint_radius`（必填；`slam_bringup/robot.launch.py` 帶入該車 global costmap 的 `robot_radius`：Nova Carter 0.81 m、Carter v1 0.71 m，與 planner、旋轉保護區同一半徑）、`max_linear_velocity`、`max_angular_velocity`（必填；`robot.launch.py` 帶入該車 `cmd_vel_safety` 的 `max_linear_speed`／`max_angular_speed`，目前兩車皆 0.75 m/s、0.5 rad/s）、`heartbeat_rate`、`battery`。
- 容器 image 需含 `paho-mqtt` 與 `py_trees`（Dockerfile 已加入）；修改後需 `docker compose run --rm ros build`。

## 擴充：多步任務（py_trees）

規格之外的擴充：`command` 的 `action` 可為 `task`，帶 `steps` 列表，由 py_trees 建出 `Sequence` 依序執行，任一步失敗就中止。

```json
{"robot_id":"carter1","cmd_id":42,"action":"task","steps":[{"type":"navigate","x":1.0,"y":2.0,"yaw":0.0}]}
```

- 步驟類型：目前只有 `navigate`（`x`、`y`、`yaw` 為 `map` 座標，`yaw` 預設 0，呼叫 `/NAME/navigate_to_pose`）。`rotate`、`take_photo` 已列入協定但尚未實作，含這些步驟的指令會被拒絕，不會執行任何步驟。
- `task_state`（retained）：`{"robot_id","cmd_id","goal_id","status","step","steps","message"}`，`status` 為 `running`、`canceling`、`succeeded`、`aborted`、`canceled`、`rejected`；`canceling` 不代表已停止，取消異常會附在 `message`。狀態改變時立即發布，完成後保留最後狀態直到下一個任務。`navigate`、`dock` 也會發。
- 結束時另發對應的 `command_result`（`succeeded`→`completed`、`aborted`／`rejected`→`failed`、`canceled`→`canceled`）。
- 新增步驟類型：在 `task_bt.py` 的 `STEP_TYPES` 加驗證函式，並在 `fleet_bridge_node.py` 的 step factory 建立對應 behaviour（`update()` 不可阻塞）。
