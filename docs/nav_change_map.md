# 導航地圖切換：操作與實作

本文件集中說明 1F／B1 換圖、RViz 操作、MQTT 接口、車內協調流程與故障排除。
一般導航架構見 [nav.md](nav.md)，其他 MQTT／RTSP 功能見 [online.md](online.md)。

## 使用條件與地圖

僅支援完整 3D 融合導航（`navigate:=true`）。2D AMCL、純定位展示與 costmap filter editor
不支援動態換圖；editor 啟用時接口明確拒絕。

導航腳本預設載入 `maps/1F/map_2d.yaml` 與同目錄的 `map.pcd`。
每層需有下列檔案，PGM 名稱依 YAML 的 `image` 欄位決定：

```text
maps/
  1F/
    map_2d.yaml
    map_2d.pgm
    map.pcd
  B1/
    map_2d.yaml
    map_2d.pgm
    map.pcd
```

`run_nav.sh --map FILE`（`run_3d_localization.sh --pgm FILE`）可改起始地圖；
未指定 `--pcd` 時使用 YAML 旁的 `map.pcd`。`floor_manager.maps_root` 預設為起始樓層目錄的上一層
（容器內通常為 `/workspace/maps`）。同層 PGM／PCD 必須共用 `map` 座標；
各層獨立建圖的座標不保證與 Isaac world 對齊，不可直接使用模擬生成位置。
只有明確選擇 Office 地圖時才套用既有 Office 自動初始化邏輯。

模擬必須持續播放並提供感測器。啟動的 TF readiness 最多等待 300 秒。

## 單車手動操作

啟動模擬、導航、RViz 與單車 MQTT 換圖 bridge：

```bash
./scripts/run_nav.sh --mqtt-host localhost
# Broker 非預設埠時加上 --mqtt-port 1884
```

1. 在 RViz 按 **2D Pose Estimate**，設定車輛在 1F 地圖上的位置及朝向，等待導航啟動。
2. 以 Nav2 目標或其他既有車端控制把車開入電梯，取消導航並確認車輛停穩。
3. 在 RViz **Floor** 面板按 **Prepare elevator travel (MQTT)**，等這筆指令回 `completed`，且 `Transition: prepared`。
4. 再點選 Isaac Sim 視窗，按 **E** 關門、**L** 換到 B1；停妥後按 **E** 開門。
5. 在 Floor 面板選擇或輸入 `B1`，按 **Switch floor (MQTT)**。
6. 等 RViz 顯示 B1 地圖，再按一次 **2D Pose Estimate**，設定車在 B1 地圖上的實際位置與朝向。
7. 等換圖指令顯示 `completed`、狀態回 `idle`，才可送 B1 的導航目標；回 1F 時重複同樣流程。

**沒有電梯互鎖。** `prepared` 只確認車端已停車並暫停導航／全域定位，
不會鎖住 E／L 按鍵，也不會阻止上位或電梯控制器移動電梯；操作順序由操作者自行注意。
準備指令的 `completed` 不代表換圖或重定位完成，更不代表已恢復導航。

**底圖改變不等於換圖完成。** 等待手動定位時，第一次請求仍在進行，不能直接再切回 1F。
必須完成定位，或等請求逾時結束，再用新指令重試。現階段沒有取消換圖或自動 rollback；
不要用假的位姿強行完成定位。

GUI 模式預設啟用 Office 手動電梯，初始關門、車廂停在地面樓層。
導航模式由 ROS `cmd_vel` 控制車輛，不啟用鍵盤駕駛。電梯操作不會自動換圖或重新定位，
也沒有防夾偵測。`--headless` 不提供手動電梯按鍵；`--no-rviz` 只關閉 RViz。
Nova Carter 的 global costmap 半徑為 0.81 m，可能讓 Nav2 無法規劃進車廂。

### 以 MQTT 指令取代面板按鈕

監聽結果與發送指令可分別在兩個終端執行：

```bash
mosquitto_sub -h localhost -q 1 -t rmf/manual/robot/carter/command_result
```

```bash
mosquitto_pub -h localhost -q 1 -t rmf/manual/robot/carter/command \
  -m '{"robot_id":"carter","cmd_id":1,"action":"prepare_floor_change"}'
```

準備指令回 `completed` 後手動操作電梯，抵達並開門後再發送：

```bash
mosquitto_pub -h localhost -q 1 -t rmf/manual/robot/carter/command \
  -m '{"robot_id":"carter","cmd_id":2,"action":"change_floor","floor_name":"B1"}'
```

省略 `initial_pose`（或設為 `null`）表示等待 RViz 手動定位。每次新操作使用新的整數 `cmd_id`，
QoS 1，**不要 retain**。換圖指令送出後仍需完成上述第 6、7 步。

### Floor 面板顯示

面板顯示目前樓層與處理結果。下拉選單預設有 `1F`／`B1`，也可輸入名稱；
清單保存在 RViz 設定的 `Floors`，不是自動掃描 `maps/`。
未加 `--mqtt-host` 時，按鈕會提示 bridge 未啟動。

進入準備或換圖階段後，面板清除並暫時停用 costmap（含膨脹區）、路徑、
多邊形（footprint／防撞區）、LaserScan（含 `Height-filtered obstacles`）與定位 pose 顯示，保留 PGM 底圖。
定位與換圖成功後才恢復原本啟用的顯示；等待定位或失敗時不恢復。
這只控制 RViz 顯示，不會停用車端碰撞保護。更新面板程式並編譯後，需重開 RViz 才生效。

## 上位系統 MQTT 接口

上位系統只需 MQTT，不必連車端 ROS 2。這是本專案 fleet bridge 的協定擴充，
不會只因 RMF level 改變就自動觸發。多車線上模式由 `fleet_bridge_node.py`
使用 `rmf/F/robot/NAME/command` 收指令，結果回 `rmf/F/robot/NAME/command_result`。
`F`／`NAME` 依車隊與車名設定，啟動方式見 [online.md](online.md)。

### `prepare_floor_change`：搭電梯前暫停全域定位

沿用同一 command topic，payload 為：

```json
{"robot_id":"carter1","cmd_id":300,"action":"prepare_floor_change"}
```

不需 `floor_name` 或 `initial_pose`。車端鎖住速度、暫停全域 pose／TF 校正與 ICP 配對、
取消舊 Nav2 目標並暫停導航，再確認新鮮里程計顯示停車；全部成功才回 `completed`。
此時進入 `prepared`，不載入新地圖，也不重設 LIO／IMU／Local EKF；車端維持收集 LIO 資料。
為避免搭梯期間仍被解讀為已確認在舊樓層，`current_floor`／結果中的 `level_name` 暫時為空。

準備狀態沒有自動解鎖時限。後續必須使用 `change_floor` 載圖並通過新定位檢查，才恢復導航；
若取消搭梯，也要用原樓層的 `change_floor` 重新確認定位，不提供無條件的「開啟定位／解除停止」指令。
重複的相同 `cmd_id` 回傳歷史快取，不代表目前仍在該歷史狀態；每次新操作使用新 ID。
新的準備指令會重新確認停車，準備與換圖不能並行。
完整 fleet bridge 若仍有執行中的任務，會拒絕準備指令，上位應先 `stop` 並等待舊任務結束。

此接口**只處理車端，不直接控制或互鎖電梯**。手動操作仍須遵守先準備、後移動電梯的順序。
`rmf_isaacsim` 的 MQTT fleet adapter 會等待進入車廂的導航指令回報對應
`command_result (completed)`，再自動發送 `prepare_floor_change`；
準備指令也回 `completed` 後，才通知 RMF 完成進梯，允許電梯關門與升降。
電梯車廂點位不以 heartbeat 座標接近判定到點；
即使目前位置已在車廂目標附近，也不跳過車端導航完成確認。
（一般點位才會在 heartbeat 距目標 0.25 m、0.2 rad 內直接完成，讓車子不必在中途點對準朝向。）
此 RMF 順序控制不會鎖住 Isaac Sim 的手動 E／L 按鍵。

### `change_floor`：抵達後換圖與重定位

帶入定位先驗的範例：

```bash
mosquitto_pub -h localhost -q 1 \
  -t rmf/fleet1/robot/carter1/command \
  -m '{
    "robot_id": "carter1",
    "cmd_id": 301,
    "action": "change_floor",
    "floor_name": "B1",
    "initial_pose": {"x": 2.1, "y": -0.8, "z": -0.526, "yaw": 1.5708}
  }'
```

**座標僅為格式範例，不是實際電梯位置。**

| 欄位 | 意義 |
|---|---|
| `floor_name` | 地圖目錄名稱，與 RMF level 一致；只接受英數字、底線、連字號，不接受路徑 |
| `initial_pose.x/y` | 目標樓層 PGM／PCD 共用 `map` 座標中的 `base_link` 位置，單位 m |
| `initial_pose.z` | 該層 PCD 中的高度，不是樓層編號或 Isaac world 高度；省略時使用 `floor_base_z`，Nova Carter 預設 -0.526 |
| `initial_pose.yaw` | `base_link` 航向，單位 rad，逆時針為正 |
| 不帶 `initial_pose`／設為 `null` | 載入目標 PGM 後等待 RViz 2D Pose Estimate；若有帶物件，`x/y/yaw` 必填，`z` 選填，數值須有限 |

車端依 robot profile 的 IMU 安裝外參與當前 LIO 位姿，換算 ICP 所需的
`map -> camera_init` 先驗，上位不用計算感測器外參。亦可送 `change_floor` 到 `1F` 初始化首次定位，
不必先取得 `register_ack`。

上位需先完成或 `stop` 舊任務並等待結果，確定電梯抵達目標層且車輛停穩後再換圖。
車廂特徵不足時先開啟目標層門，讓 LiDAR 看見環境。接口不控制電梯、樓層感測或車輛高度；
換圖期間不得移動或模擬傳送車輛，初始位姿不會繞過定位品質檢查。

`rmf_isaacsim` 的 MQTT fleet adapter 已實作：RMF 乘電梯（`--elevator-lift lift_1`）
抵達目的樓層開門後發送只帶 x/y/yaw 的 `change_floor`。`initial_pose` 取自 `prepare_floor_change`
結果 `final_location`（凍結定位當下的實際位姿；單車 bridge 沒有此欄位時改用進梯導航結果的位姿），
先以出發樓層 `levels[].transform` 轉回 RMF 建築座標，再以目的樓層 transform 轉到目的地圖；
yaw 同步加上兩層 rotation 差。若換算結果距 RMF 車廂點超過 1.5 m，改用車廂點位置但保留換算後的 yaw；
沒有記錄位姿時退回 RMF 車廂點位姿。
電梯通訊見 [isaac_bridge.md](isaac_bridge.md)。Office 電梯位置與 B1 transform 取自 Isaac 參考位姿，
未經實車校正，可能有公分級誤差。

### 結果、重送與逾時

結果沿用原 `cmd_id`：

```json
{
  "robot_id": "carter1",
  "cmd_id": 301,
  "status": "completed",
  "message": "Floor B1 maps, ICP and navigation ready",
  "level_name": "B1",
  "final_location": {"x": 2.1, "y": -0.8, "yaw": 1.5708},
  "timestamp": 1792036000.0
}
```

只有 **`change_floor` 的 `completed`** 才能繼續導航。`prepare_floor_change` 的 `completed` 僅代表準備完成。
完整 fleet bridge 的 `heartbeat.level_name` 同步反映樓層，
指令執行中 `status: moving`、`current_cmd_id` 為該指令；準備完成但尚未換圖時仍非 idle，
回報 `moving`，沒有執行中的指令時 ID 為 `null`。準備完成或已開始部分換圖時，樓層暫時未知
（結果的 `level_name` 為空字串，heartbeat 省略此欄位）。單車 bridge 不送 register／heartbeat，
也不提供上述範例的 `final_location`。

執行中的相同 `cmd_id` 不重複執行；最近 64 筆換圖結果會重送快取，重啟後不保留。
修正問題後用**新 `cmd_id`** 重試，也可切回原樓層，但不是自動 rollback。
已開始切換後若失敗，車輛保持停止、Nav2 暫停；fleet bridge 在換圖中或失敗後拒絕其他命令
（含 `stop`），直到成功重試；準備完成後也不接受一般任務。
單車 bridge 只接受 `prepare_floor_change`／`change_floor`，其他 action 一律拒絕。
參數／檔案預檢失敗不改動原地圖或安全鎖，但 fleet bridge 仍要求成功重試才接受其他命令。
緊急停止是獨立且不可被此接口解除的車內安全鎖。

| 參數 | 預設值與用途 |
|---|---|
| `floor_manager.switch_timeout` | 60 秒，車內換圖操作時限 |
| `floor_manager.service_timeout` | 10 秒，個別服務呼叫時限 |
| `manual_pose_timeout` | 300 秒，等待手動定位；manager 與 bridge 應保持一致 |
| bridge 的 `floor_switch_timeout` | 90 秒；手動模式加上 `manual_pose_timeout`，預設共 390 秒 |

手動模式在 PGM 載入後額外等待定位，收到 pose 後重新給予車內換圖操作時限。
bridge 逾時回 `failed` 後仍追蹤未完成請求，避免再送並行操作；延遲結果不改寫已回報的失敗結果。

## 實作方式

### 指令路徑

```text
RViz FloorPanel
  → ROS navigation/floor_prepare_request（準備）或 navigation/floor_request（樓層名稱）
  → floor_command_bridge.py（產生新的時間戳整數 cmd_id）
  → MQTT broker：rmf/manual/robot/carter/command
  → floor_command_bridge.py 訂閱收回
  → ROS navigation/prepare_floor_change 或 navigation/change_floor
  → floor_manager.py
```

面板不是直接呼叫換圖 service，也不是直接連 broker；因此按鈕仍測到完整 MQTT 路徑。
bridge 以 `navigation/floor_command_status` 回報文字狀態。
上位自動模式則是 RMF → MQTT → `fleet_bridge_node.py` → 同一車內 service。

### 換層狀態

manager 以 transient-local 的 `navigation/floor_transition_state` 發布狀態：

```text
idle → preparing → prepared → switching → idle
           ↓                     ↓
         failed ←────────────────┘
```

`failed` 可重新準備或換圖；參數／地圖預檢失敗不改變既有狀態。
為相容既有上位，也仍接受直接從 `idle` 進入 `change_floor`，但這不會保護先前的搭梯過程；
建議採用準備／換圖兩階段。只有成功換圖才解除速度鎖。

### 車內切換順序

1. 驗證樓層檔案與位姿，鎖住速度輸出與全域 pose／TF／ICP，再取消舊 Nav2 目標並暫停導航，確認新鮮里程計顯示停車。若已準備完成仍會重新確認。
2. 將 `navigation/current_floor` 發為空字串，標示地圖配對尚未確認；Floor 面板同時依換層狀態隱藏導航疊圖。
3. 透過 `map_server/load_map` 載入目標 PGM。
4. 若未帶位姿，等待在開始載入 PGM 後收到的 RViz `initialpose`；忽略較早的 pose，z 為 0 時改用 `floor_base_z`。
5. 透過 `localization_3d/load_map` 載入 PCD 與換算後先驗，再釋放全域 pose 與 ICP 的 hold；等待新 ICP pose，要求距先驗小於 1 m、航向差小於 0.5 rad。
6. 以接受的 pose 重設 Global EKF，保留連續 Local EKF／LIO；釋放 TF 校正並確認新鮮、匹配的全域 TF。
7. 恢復 Nav2、清除兩張 costmap，再次確認定位；最後解除換圖速度鎖、發布成功樓層並回 `completed`。Floor 面板清除顯示快取並恢復原本開啟的導航疊圖。

RViz 的 `initialpose` 統一表示 `map -> base_link`；localizer 與 PCD load service 使用相同換算：
`T_map_camera_init = T_map_base × inverse(T_body_base) × inverse(T_camera_init_body)`，
避免把搭電梯後已累積的 LIO 位移再次加到手動定位上。RViz z 為 0 時使用地圖的地面車身高度。
Office 自動初始化也先轉成 `map -> base_link` 再發布，保持既有 spawn 先驗的意義。
準備與等待手動 pose 期間，`localization_3d/set_registration_hold` 暫停全域 ICP；
localizer 不對舊地圖處理 RViz click，由 manager 收下位姿並載入目標 PCD，不再依賴固定 1 秒延遲。
ICP 確認期間不要重按 2D Pose Estimate，以免改變換圖請求的定位先驗；若失敗應重送新的換圖指令。

### 程式位置

| 路徑（相對 repository root） | 責任 |
|---|---|
| `scripts/run_nav.sh`、`scripts/run_3d_localization.sh` | `--mqtt-host`／`--mqtt-port` 與啟動參數 |
| `ros2_ws/src/slam_localization_3d/launch/global_fusion.launch.py` | 啟動 manager、選用的單車 bridge 與 RViz |
| `ros2_ws/src/slam_localization_3d/src/floor_panel.cpp` | 樓層按鈕、狀態、導航疊圖隱藏／恢復 |
| `ros2_ws/src/slam_localization_3d/config/global_fusion.rviz` | 預設 Floor 面板與樓層清單 |
| `ros2_ws/src/slam_fleet_bridge/scripts/floor_command_bridge.py` | 單車 MQTT 收發、ROS 面板轉發與結果快取 |
| `ros2_ws/src/slam_fleet_bridge/scripts/fleet_bridge_node.py`、`rmf_protocol.py` | 完整 fleet 指令、協定解析與換圖 request 組裝 |
| `ros2_ws/src/slam_localization_3d/scripts/floor_manager.py`、`floor_maps.py` | 切換協調、等待條件與地圖／位姿檢查 |
| `ros2_ws/src/slam_localization_3d/srv/ChangeFloor.srv` | 請求含 `floor_name`、`initial_pose`、`manual_initial_pose`；回應含成功與否、訊息、目前樓層與最終位姿 |
| `ros2_ws/src/slam_localization_3d/srv/PrepareFloorChange.srv` | 空請求；回應含成功與否、訊息與目前樓層，只準備而不換圖／解鎖 |
| `ros2_ws/src/slam_localization_3d/scripts/global_localization_xyz.py` | PCD 載入、ICP 與位姿先驗換算 |
| `tests/test_floor_switching.py`、`tests/test_rmf_protocol.py`、`tests/test_script_entrypoints.py` | 車內流程、MQTT、協定與入口腳本測試 |
| `ros2_ws/src/slam_localization_3d/test/test_floor_panel.cpp` | GUI 按鈕、狀態、重複換層與導航疊圖恢復測試 |

上述 ROS service 僅供車內協調；多車時加 `/NAME` namespace，維持各車 TF／topic 隔離。
Floor 面板的自動疊圖控制針對單車 `global_fusion.rviz`；共享的多車 RViz 面板不等同於此面板。

## 故障排除

| 現象 | 原因與處理 |
|---|---|
| 地圖換了卻不能再換／顯示 `Another floor switch is in progress` | 上次操作尚未結束，常見是還沒設定 2D Pose Estimate；完成定位或等逾時結束再重試 |
| 不定位直接想切回原層 | 目前不支援取消進行中的換圖；需等該請求結束，再發新指令，且仍需確認定位 |
| 一直等不到 `completed` | 確認模擬播放、車已停穩、電梯已開目標層門、pose 座標與目標 PCD 一致；檢查 manager 的 ICP／TF／服務逾時訊息 |
| `accepted target-floor ICP pose near initial_pose` 逾時 | 查看結果中的 `target`、`latest pose`、`distance`、`yaw_error`、`age` 與 `newer_than_load`；沒有 pose 代表未收到接受的定位，不等於單純點錯位置。有 pose 則確認距離小於 1 m、角度差小於 0.5 rad，且為載圖後的新鮮資料 |
| 按面板沒有連到 MQTT | 以 `--mqtt-host HOST` 啟動，確認 broker 可連線；面板會顯示 bridge 未啟動或 broker 未連線 |
| 失敗後重送沒有重新執行 | 不要沿用已回覆結果的 `cmd_id`；面板每次按下會自動產生新的 ID |
| 重按按鈕後看不到等待定位提示 | 新指令的 busy 拒絕訊息會覆蓋原提示，不代表原請求已取消 |
| 換圖時還看到舊膨脹區／路徑 | 確認使用更新後的 Floor 面板，並重開 RViz；切換期間這些顯示應暫時停用 |
| 失敗後導航疊圖不出現、車也不動 | 安全設計：地圖與定位尚未確認，不恢復導航；修正原因並重新換圖成功後才恢復 |
| 準備已 `completed` 卻還不能導航 | 正常；此時只完成準備，必須再以 `change_floor` 確認定位成功才解鎖 |
| ICP held 時按 2D Pose Estimate 沒反應 | 準備狀態不直接重新定位；先發 `change_floor`，等新底圖出現後再設定 pose |
| 準備尚未完成就能按 E／L | 沒有電梯互鎖；請自行確認 `prepared` 後才操作電梯 |

### 搭電梯後的重定位座標修正

舊版直接將 RViz `map -> base_link` 當作 `map -> camera_init`，忽略車輛已累積的
LIO 位移與安裝外參；即使電梯門打開、位置點選正確，也可能從錯誤位置開始 ICP。
現已統一使用上述座標換算，並同步調整 Office 自動初始化的發布端，沒有降低 `0.8` 品質門檻。

驗證包含非零平移、旋轉、下降 3.3 m 的合成點雲配對，以及實際模擬 B1 掃描的唯讀離線配對：
同一筆掃描與由建圖參考位姿估算的先驗，舊換算 fitness 約 `0.735`，修正後為 `1.0`。
這是單次快照的定位驗證，不代表任意位置／先驗都能成功，也不等於整段電梯導航已重跑。
更新並編譯後需重啟導航程序，既有執行中的 Python node 不會自動載入修正。
