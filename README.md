# Isaac Sim + FAST-LIO2 + Nav2

使用 Isaac Sim 模擬 Nova Carter，支援 FAST-LIO2 建圖、2D／3D 定位與 Nav2 導航。多車模式採用**一車一個導航 container，以及一個共用 RViz**。

## 1. 建置專案環境

### 環境需求

- Linux、NVIDIA GPU 與可用的 NVIDIA 驅動。
- Docker、Docker Compose v2、NVIDIA Container Toolkit。
- 已安裝 Isaac Sim 6.1.0（本文以 `<ISAACSIM_PATH>` 代表安裝目錄，例如 `~/isaacsim-6.1.0`，實際路徑依各機器而異）。目前 `scripts/standalone.py`、`scripts/usd_bbox.py` 第一行的 shebang 寫死了某個安裝路徑，首次使用前請改成 `#!<ISAACSIM_PATH>/python.sh`；`tests/` 內的驗證腳本則可用環境變數 `ISAAC_PYTHON=<ISAACSIM_PATH>/python.sh` 覆寫。
- GUI 模式需要可用的 X11／XWayland 顯示環境及 `DISPLAY`；若使用 Xauthority 驗證，需設定 `XAUTHORITY` 指向可用的授權檔。
- Isaac Sim 的 Office／Nova Carter 資產需可存取。

Isaac Sim 在主機執行，ROS 2 Humble、FAST-LIO2、Nav2 和 RViz 在 Docker 內執行，**主機不必另外安裝 ROS 2**。

```bash
git clone --recurse-submodules https://github.com/piggaycheng/isaacsim_fast-lio2-slam.git
cd isaacsim_fast-lio2-slam

# 已 clone 的專案也可用這行補齊 submodule
git submodule update --init --recursive

export HOST_UID="$(id -u)"
export HOST_GID="$(id -g)"
docker compose build
docker compose run --rm ros build
```

修改 `ros2_ws/src` 後重新執行 workspace build 即可；修改 Docker 安裝依賴時，才需要重建 image：

```bash
docker compose run --rm ros build
# 只建置指定套件
docker compose run --rm ros build --packages-select slam_localization_3d
```

導航與定位套件使用中性名稱：`slam_nav`、`slam_localization_2d`、`slam_localization_3d`。`isaac_fastlio_adapter` 仍是 Isaac Sim 專用的感測器轉接套件；目前 launch 與預設配置仍針對模擬，改名不代表可直接套用到實機。從舊名稱升級的工作區，需移除這三個舊套件對應的 `ros2_ws/build/` 與 `ros2_ws/install/` 子目錄後重新建置，避免舊套件仍被 ROS 找到。

## 2. 各個 sh 如何使用

專案級啟動腳本與 Python 工具集中在 `scripts/`；ROS 套件、`docker/` 與 `tests/` 內的腳本維持原位。以下指令皆在**專案根目錄**執行，地圖等資料仍放在 `maps/`。一般啟動腳本會自動啟動 ROS container、RViz 與 Isaac Sim，按 **Ctrl+C** 結束並清理 container。除多車模式外，不要同時啟動多套建圖／定位／導航腳本。

### 建圖與存圖

```bash
./scripts/run_slam.sh
```

啟動 Isaac Sim、RViz 與建圖管理器；**初始為閒置，尚未啟動 FAST-LIO2／PGO**。自動啟用 Office 電梯的手動模式（初始關門，車廂停在地面樓層）。使用 **W/S/A/D 或方向鍵**操控車輛，**Space** 停車；點選 Isaac Sim 模擬視窗後按 **E** 同步開／關停靠樓層的電梯門，按 **L** 同步切換兩個車廂的上下樓層。

進入車廂後先按 **Space** 停車，再按 **L**；電梯會先關門、用約 6 秒升降，抵達後按 **E** 開門。井口不再有透明碰撞地板，未停靠樓層的門保持關閉；目前不含防夾偵測或自動電梯任務協調。導航的樓層換圖接口見下方及[對外連線文件](docs/online.md#mqtt-換樓層與初始定位)。

等模擬載入完成、保持播放並停穩車輛，另開終端控制每層的獨立建圖：

```bash
./scripts/slam_session.sh start floor_ground  # 全新的 FAST-LIO2／PGO；等回覆 mapping 再行駛
./scripts/slam_session.sh status
./scripts/slam_session.sh save-stop          # 存圖成功後停止建圖；模擬與 RViz 不關閉
# 搭電梯換樓層，出電梯後停穩，再開始下一層
./scripts/slam_session.sh start floor_lower
./scripts/slam_session.sh save-stop
```

地圖分別存到 `maps/floor_ground`、`maps/floor_lower`，包含 `map.pcd`、`poses.txt`、`patches/`、`session.json` 與 `slam.log`。每個 `start` 都重建估計器與地圖，**不支援接續舊地圖**；已存在的樓層目錄會拒絕使用，可改用 `floor_ground_run2` 等新名稱。存圖失敗時建圖繼續運行，可檢查狀態及 log 後重試 `save-stop`。退出 `run_slam.sh` 前先 `save-stop`，Ctrl+C 不會自動存圖。完整操作、座標注意事項與故障排除見[建圖管理器使用指南](docs/mapping_manager.md)。

若只要額外快照、不停止目前建圖，原有存圖工具仍可使用：

```bash
./scripts/save_map.sh
./scripts/save_map.sh --output-dir maps/office --voxel-size 0.05 --save-patches true
```

`save_map.sh` 預設輸出到 `maps/office`（與管理器的樓層目錄不同），可能覆寫該目錄中的快照。導航所需的 2D 地圖可再由各層 PCD 轉換，例如：

```bash
./scripts/pcd2pgm.py maps/floor_ground/map.pcd maps/floor_ground/map_2d
```

### 單車定位與導航

| 腳本 | 用途與範例 |
|---|---|
| `scripts/run_nav.sh` | 預設 3D 融合定位與完整 Nav2 導航；`--mode 2d` 保留 AMCL 純定位 |
| `scripts/run_2d_localization.sh` | 2D AMCL 定位；例如 `./scripts/run_2d_localization.sh --map maps/office/map_2d.yaml` |
| `scripts/run_3d_localization.sh` | 3D ICP 定位顯示；加 `--global-fusion` 啟動輪速／IMU／PCD 融合 |

```bash
./scripts/run_nav.sh --mode 2d                 # AMCL 定位，不啟動自主導航
./scripts/run_nav.sh                           # 預設 3D 融合定位與完整 Nav2 導航
./scripts/run_nav.sh --mode 3d                 # 同上，不需再加 --navigate
./scripts/run_3d_localization.sh              # 獨立的 3D 定位顯示
./scripts/run_3d_localization.sh --global-fusion --costmaps # 融合與 costmap 觀察，不導航
```

預設 `maps/1F` 先用 RViz 的 **2D Pose Estimate** 設定初始位置與航向；等定位收斂、導航堆疊啟動完成後，用 **2D Goal Pose** 在地圖上按住左鍵拖曳，設定目標位置與航向。啟動不會自動發送導航目標；舊的 `--navigate` 參數仍相容。其他導航選項（`--box`、`--filter-editor`、`--static-zones` 等）見[導航文件](docs/nav.md)。

導航／定位腳本預設地圖為 **`maps/1F`**，初始定位需使用 RViz 或線上模式的 MQTT `change_floor`。
上位系統可用 `floor_name: "B1"` 與目標樓層 `base_link` 的 `initial_pose: {x, y, z, yaw}`
通知車端一起切換 PGM／PCD、重新定位；僅收到 `command_result.status: completed` 後才繼續導航。
上位不需要 ROS 2，完整 payload、停車及失敗處理見[MQTT 換樓層接口](docs/online.md#mqtt-換樓層與初始定位)。

常用參數：

- `--headless`：關閉 Isaac GUI；`--no-rviz`：關閉 RViz，兩者可一起使用。
- `--map FILE`：指定地圖 YAML，不是直接指定 PGM；直接使用 `scripts/run_3d_localization.sh` 時改用 `--pgm FILE`。
- `--pcd FILE`：指定 3D PCD 地圖（3D 模式）。
- `--manual-initial-pose`：改用 RViz 的 **2D Pose Estimate** 手動初始化（3D 模式）。

### 多車導航

```bash
./scripts/run_multi_nav.sh                     # carter1：Nova Carter；carter2：Carter v1
./scripts/run_multi_nav.sh --robot carter1@0,0 --robot carter2:carter_v1@3.5,0,1.57
./scripts/run_multi_nav.sh --robot a@0,0 --robot b@3.5,-2,1.57 --robot c@1,2
```

`--robot NAME[:TYPE][#FLEET]@X,Y[,YAW]` 可重複指定（位置單位為公尺、航向為弧度，省略 TYPE 為 `nova_carter`；`FLEET` 為 Open-RMF 車隊，見[對外連線文件](docs/online.md)）；RViz 的 **Fleet Control** 選車後送目標。支援 `--map`、`--pcd`、`--headless`、`--no-rviz` 等參數。架構、車種設定與限制見[多車文件](docs/multi_robot.md)。

各腳本完整參數可用 `--help` 查看；`scripts/run_slam.sh` 沒有參數介面。

### 內部腳本

| 腳本 | 用途與範例 |
|---|---|
| `docker/build_workspace.sh` | 容器內建置流程，通常透過 `docker compose run --rm ros build` 使用 |
| `docker/entrypoint.sh` | 容器入口，載入 ROS 環境並執行命令，不需從主機直接啟動 |
| `docker/ros_compose.sh` | 啟動腳本共用的 Compose helper，供其他腳本 `source` 使用 |

## 3. 更換車種：量測 footprint

換成非 Nova Carter 的車種時，不能沿用 Carter 的車體尺寸、輪徑、輪距、感測器外參與 covariance。車種 profile 的建立方式見[新增車種](docs/multi_robot.md#新增車種)，covariance 校正見[協方差校正](docs/covariance_calibration.md)。

`scripts/usd_bbox.py` 可從 USD 模型量測車體 bounding box，輸出 Nav2 footprint。在主機使用 Isaac Sim 的 Python launcher，將路徑與 frame 換成新車的模型及 `base_link` 對應的 USD prim：

```bash
<ISAACSIM_PATH>/python.sh scripts/usd_bbox.py /path/to/new_robot.usd \
  --frame base_link --yaw-deg 0 --padding 0.06
```

`--yaw-deg` 須符合 USD frame 與 ROS `base_link` 的方向差（Nova Carter 與 Carter v1 都以差速輪端為車頭，朝 USD +x，使用 0 度）；`--padding 0.06` 只是示例；`--shape hull` 可改為凸包，`--json` 輸出完整結果。沒有 USD 模型的實機需以實測尺寸建立 footprint。

將 footprint 同步填入新車 profile 的 `parameter_overrides`：global／local costmap（`observation_costmaps.yaml`）、`ground_obstacle_filter` 的 `self_filter_bounds`，以及 `collision_monitor` 的停止／減速區域，不要改動共用預設值。工具不會自動寫入設定，也不會量測煞停距離，修改後仍需驗證碰撞停止行為。

## 詳細文件

- [專案概觀：流程、設計理念與疑難排解](docs/overview.md)（建議先讀）
- [Isaac Sim 橋接與 Docker 運作](docs/isaac_bridge.md)
- [建圖流程](docs/mapping_flow.md)
- [3D 定位架構](docs/3d_localization.md)
- [導航、安全限制與驗證](docs/nav.md)
- [多車架構、車種設定與部署](docs/multi_robot.md)
- [對外連線：MQTT 與 RTSP 相機](docs/online.md)
- [更換車輛後的協方差校正](docs/covariance_calibration.md)
