# 對外連線（MQTT／RTSP）

`./scripts/run_multi_nav_online.sh` 與 `run_multi_nav.sh` 的導航、多車參數相同（見[多車文件](multi_robot.md)），另外預設啟動會對外的連線：MQTT client（相機列表與開關）與 RTSP 相機串流。`run_multi_nav.sh` 不含這些功能，也不需要 broker。

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

- **天花板固定相機**（預設提供，`--no-ceiling-cameras` 關閉）：`ceiling_cam_1`～`4` 位於 Office 座標 (−14, 10)、(4.5, 10)、(4.5, −10)、(−14, −10)，高 2.9 m，朝 (0, 0, 0) 地板。開關 topic `slam/cameras/ceiling_cams/ceiling_cam_N/enable`，串流 `rtsp://HOST:8554/ceiling_cams/ceiling_cam_N`。位置與高度在 `standalone.py` 的 `CEILING_CAMERA_*` 常數。
- **車載相機**：`--camera [ROBOT=]CAMERA`（可重複）。`CAMERA` 是相對於車輛 prim 的相機路徑或絕對 prim 路徑；`NAME` 為路徑去掉 `chassis_link/sensors/` 後以 `_` 連接，如 `front_owl_camera`。開關 topic `slam/cameras/[ROBOT/]camera/NAME/enable`，串流 `rtsp://HOST:8554/[ROBOT/]camera/NAME`。設有 `simulation.gimbal` 的車（見 [multi_robot.md](multi_robot.md)）在指定 `--mqtt-host` 時，雲台相機自動加入列表，名稱 `gimbal`（`camera/gimbal`），同樣預設關閉、收到 enable 才串流，不需 `--camera`；雲台姿態控制見 multi_robot.md。Nova Carter 的相機：`front_owl`／`left_owl`／`right_owl`／`back_owl`（`.../camera`）及 `front_hawk` 等（`.../left/camera_left`）。
- **MQTT**：`--mqtt-host`（預設 `$MQTT_HOST` 或 `localhost`）、`--mqtt-port`（1883）、`--mqtt-topic`（`slam/cameras`）、`--mqtt-username/--mqtt-password`。相機列表以 retained JSON 發布在 `slam/cameras`（`name`（識別用，同 `rtsp_path`）、`display_name`（給介面顯示，如 `Ceiling camera 1`、`a gimbal`）、`rtsp_path`、`enable` topic、解析度、fps、`enabled`），僅在相機開關變動時更新，結束時發布 `{"online":false}`（異常斷線由 LWT 發布）。Broker 可晚於模擬啟動，client 會在背景重連。需在 Isaac Sim Python 安裝 `paho-mqtt`。
- `--camera-resolution WxH`（預設 640x480）、`--camera-fps N`（預設 15，最高 60）。
- RTSP server 是 MediaMTX（Docker image `bluenviron/mediamtx:latest-ffmpeg`，host network，埠 8554），第一次開啟相機時才啟動；編碼用同一 image 內的 ffmpeg（`libx264`）。首次使用會自動下載 image。瀏覽器不能直接播 RTSP，網頁需另行轉成 WebRTC／HLS（MediaMTX 也提供，埠 8889／8888，但此專案未設定）。

## 車輛位置轉發（MQTT，給 fleet adapter）

`run_multi_nav_online.sh` 會在每台車的容器內啟動 `slam_fleet_bridge` 套件的 `fleet_bridge_node.py`（namespace `/NAME`）。每台車只用**一條** MQTT 連線，同時負責位置轉發與下方的任務指令，讓外部 fleet adapter 不必使用 ROS。位置部分把 `/NAME/odometry/global`（`map` 座標）轉成 MQTT：

| Topic | 內容 |
| :-- | :-- |
| `fleet/NAME/state` | JSON `{"robot","frame_id","stamp","x","y","yaw"}`，QoS 0，每秒最多 2 次 |
| `fleet/NAME/online` | retained，連線後為 `true`，離線或異常斷線（LWT）為 `false` |

- Topic 前綴用 `--fleet-topic-prefix P`（或 `$FLEET_TOPIC_PREFIX`）修改；broker 使用與相機相同的 `--mqtt-host/--mqtt-port/--mqtt-username/--mqtt-password`。
- `stamp` 是模擬時間（秒），`yaw` 單位為 rad。
- 容器 image 需含 `paho-mqtt`：更新後先 `docker compose build ros`，再 `docker compose run --rm ros build`。
- 單獨使用：`ros2 launch slam_localization_3d robot.launch.py ... mqtt_host:=HOST`（`mqtt_host` 為空時不啟動）。

## 車端任務 Behavior Tree（py_trees）

同一個 `fleet_bridge_node.py` 會把 fleet adapter 的指令交給 py_trees 依序執行（`robot.launch.py` 參數 `task_bt:=false` 可關閉任務功能，只保留位置轉發）。指令由 `task_bt.py` 解析並建出 `Sequence`，各步驟依序執行，任一步失敗就中止。

| Topic | 方向 | 內容 |
| :-- | :-- | :-- |
| `fleet/NAME/command` | adapter → 車（QoS 1，不要 retained） | `{"goal_id":"42","steps":[{"type":"navigate","x":1.0,"y":2.0,"yaw":0.0}]}` |
| `fleet/NAME/cancel` | adapter → 車（QoS 1） | `{"goal_id":"42"}`（`goal_id` 可省略，取消目前任務） |
| `fleet/NAME/task_state` | 車 → adapter（QoS 1，retained） | `{"robot","goal_id","status","step","steps","message"}` |

- `status`：`running`、`succeeded`、`aborted`、`canceled`、`rejected`。狀態改變時立即發布，執行中每秒重發一次；完成後保留最後狀態直到下一個任務。
- 步驟類型：目前只有 `navigate`（`x`、`y`、`yaw` 為 `map` 座標，`yaw` 預設 0，呼叫 `/NAME/navigate_to_pose`）。`rotate`、`take_photo` 已列入協定但尚未實作，含這些步驟的指令會被 `rejected`，不會執行任何步驟。
- 同一個 `goal_id` 重複送出會被忽略；不同 `goal_id` 會先取消目前任務（含 Nav2 目標）再開始新任務。
- `rejected`（格式錯誤或步驟尚未實作）只回報該指令，不影響執行中的任務，所以 adapter 要用 `goal_id` 對應狀態。
- 新增步驟類型：在 `task_bt.py` 的 `STEP_TYPES` 加驗證函式，並在 `fleet_bridge_node.py` 的 step factory 建立對應 behaviour（`update()` 不可阻塞）。
- 容器 image 需含 `py_trees`（Dockerfile 已加入）。
