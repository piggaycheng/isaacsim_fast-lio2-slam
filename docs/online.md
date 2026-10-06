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
- **車載相機**：`--camera [ROBOT=]CAMERA`（可重複）。`CAMERA` 是相對於車輛 prim 的相機路徑或絕對 prim 路徑；`NAME` 為路徑去掉 `chassis_link/sensors/` 後以 `_` 連接，如 `front_owl_camera`。開關 topic `slam/cameras/[ROBOT/]camera/NAME/enable`，串流 `rtsp://HOST:8554/[ROBOT/]camera/NAME`。Nova Carter 的相機：`front_owl`／`left_owl`／`right_owl`／`back_owl`（`.../camera`）及 `front_hawk` 等（`.../left/camera_left`）。
- **MQTT**：`--mqtt-host`（預設 `$MQTT_HOST` 或 `localhost`）、`--mqtt-port`（1883）、`--mqtt-topic`（`slam/cameras`）、`--mqtt-username/--mqtt-password`。相機列表以 retained JSON 發布在 `slam/cameras`（名稱、`rtsp_path`、`enable` topic、解析度、fps、`enabled`），僅在相機開關變動時更新，結束時發布 `{"online":false}`（異常斷線由 LWT 發布）。Broker 可晚於模擬啟動，client 會在背景重連。需在 Isaac Sim Python 安裝 `paho-mqtt`。
- `--camera-resolution WxH`（預設 640x480）、`--camera-fps N`（預設 15，最高 60）。
- RTSP server 是 MediaMTX（Docker image `bluenviron/mediamtx:latest-ffmpeg`，host network，埠 8554），第一次開啟相機時才啟動；編碼用同一 image 內的 ffmpeg（`libx264`）。首次使用會自動下載 image。瀏覽器不能直接播 RTSP，網頁需另行轉成 WebRTC／HLS（MediaMTX 也提供，埠 8889／8888，但此專案未設定）。

## 車輛位置轉發（MQTT，給 fleet adapter）

`run_multi_nav_online.sh` 會在每台車的容器內啟動 `mqtt_pose_bridge.py`（namespace `/NAME`），把 `/NAME/odometry/global`（`map` 座標）轉成 MQTT，讓外部 fleet adapter 不必使用 ROS：

| Topic | 內容 |
| :-- | :-- |
| `fleet/NAME/state` | JSON `{"robot","frame_id","stamp","x","y","yaw"}`，QoS 0，每秒最多 2 次 |
| `fleet/NAME/online` | retained，連線後為 `true`，離線或異常斷線（LWT）為 `false` |

- Topic 前綴用 `--fleet-topic-prefix P`（或 `$FLEET_TOPIC_PREFIX`）修改；broker 使用與相機相同的 `--mqtt-host/--mqtt-port/--mqtt-username/--mqtt-password`。
- `stamp` 是模擬時間（秒），`yaw` 單位為 rad。
- 容器 image 需含 `paho-mqtt`：更新後先 `docker compose build ros`，再 `docker compose run --rm ros build`。
- 單獨使用：`ros2 launch slam_localization_3d robot.launch.py ... mqtt_host:=HOST`（`mqtt_host` 為空時不啟動）。
- 目前只轉發位置；下達目標、取消與狀態回報尚未實作。
