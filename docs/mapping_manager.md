# 分樓層建圖管理器使用指南

第一版管理器讓你在同一次 Isaac Sim 執行期間，逐層建立獨立地圖。每層開始時重新啟動 FAST-LIO2／PGO，結束時先存圖，再停止建圖；模擬、電梯、感測器與 RViz 不會因為切換建圖 session 而關閉。

**這不是暫停後續掃同一張地圖。每次 `start` 都是全新地圖，不能恢復舊 session，也不會自動對齊不同樓層的座標。**

## 1. 執行前確認

- 以下指令都在專案根目錄執行。
- 使用既有的 Docker ROS 環境，workspace 需已建置完成；本功能不需另外安裝主機端 ROS Python 套件或重新建置 image。
- 第一版支援 `run_slam.sh` 的單台預設 Nova Carter。不要同時執行另一套建圖、定位或導航 launcher。
- 初始化建圖時，Isaac Sim 必須保持播放，車子必須停穩。

## 2. 啟動模擬

在第一個終端執行並保持運行：

```bash
./scripts/run_slam.sh
```

這會啟動 Isaac Sim、RViz 與建圖管理器。**起始狀態是 `idle`，尚未建圖**，必須另外執行 `start`。

RViz 使用 `/clock` 模擬時間（`use_sim_time=true`），與點雲及 TF 的時間一致。管理器在初始化的 30 秒內保留 ground truth，收到第一筆 LIO 里程計後立即鎖定時間相符的參考位姿，避免 DDS 發現或傳輸延遲導致早期 ground truth 被淘汰。

等 Office 場景與車輛載入完成後，點選 Isaac Sim 模擬視窗，使用以下按鍵：

| 按鍵 | 功能 |
|---|---|
| W/S/A/D 或方向鍵 | 操控車輛 |
| Space | 停車 |
| E | 開／關停靠樓層的電梯門 |
| L | 切換電梯上下樓層；先關門，再升降 |

電梯初始停在地面樓層。升降約需 6 秒，抵達後按 E 開門；未停靠樓層的門不能開啟。建圖管理指令在終端執行，沒有額外的建圖快捷鍵。

## 3. 建立第一層地圖

在第二個終端執行：

```bash
./scripts/slam_session.sh start floor_ground
```

`floor_ground` 是你指定的 session／樓層名稱，不是管理器自動偵測的樓層。指令會建立 `maps/floor_ground/`，啟動 FAST-LIO2／PGO，等待里程計、存圖服務與時間接近的模擬 ground truth 就緒。

**等 JSON 回覆中的 `"ok": true`、`"state": "mapping"` 後才開始行駛掃圖。** 初始化期間不要移動車子，也不要暫停模擬。

隨時可以查詢：

```bash
./scripts/slam_session.sh status
```

回覆包含 `state`、目前的 `session`、上一個 `last_session` 與最近的 `last_error`。指令序列化處理；若另一個初始化或存圖指令尚未結束，`status` 會等待它完成。

## 4. 存圖並停止建圖

完成這一層後先停車，再執行：

```bash
./scripts/slam_session.sh save-stop
```

管理器會呼叫 `/pgo/save_maps`，檢查存圖成功、主 PCD 的 binary 資料長度、poses 與 patches，寫入 metadata 後才停止建圖程序。

成功時回覆 `"state": "idle"`，且 `last_session.state` 為 `"saved_stopped"`。**確認成功後才搭電梯換樓層。** Isaac Sim、電梯、感測器、模擬時鐘與 RViz 仍會運行。

若存圖失敗或服務回覆逾時，建圖不會因此被停止。先檢查 `status` 與 log，再重試同一個 `save-stop`；逾時但尚未完成的服務請求會繼續等待，不重複發送存圖請求。

## 5. 搭電梯並建立下一層地圖

確認上一層已存圖並回到 `idle` 後：

1. 在模擬視窗按 E 開門，將車子駛入車廂。
2. 按 Space 停車，再按 L 升降；抵達後按 E 開門。
3. 駛出車廂，在下一層停穩，執行新的建圖指令。

```bash
./scripts/slam_session.sh start floor_lower
# 等回覆 mapping 後掃圖
./scripts/slam_session.sh save-stop
```

不要在同一個建圖 session 中搭電梯換樓層，也不要只切斷 LiDAR／IMU 輸入來代替停止估計器。

若要重新掃同一層，使用新的名稱：

```bash
./scripts/slam_session.sh start floor_ground_run2
```

名稱允許 1～64 個英數字、底線或連字號，第一個字必須是英數字。**已存在的目錄一律拒絕使用**，包括初始化失敗留下的目錄；不提供覆寫或恢復模式。

## 6. 輸出檔案與座標

每層各自輸出：

```text
maps/
  floor_ground/
    map.pcd
    poses.txt
    patches/
    session.json
    slam.log
  floor_lower/
    ...
```

| 檔案 | 用途 |
|---|---|
| `map.pcd` | PGO 優化後的地圖 |
| `poses.txt` | 關鍵幀位姿 |
| `patches/` | 關鍵幀點雲 |
| `session.json` | 樓層名稱、時間、狀態、點數、初始 LIO 位姿、對應 Isaac chassis 位姿及感測器外參 |
| `slam.log` | 該次建圖的 ROS launch／節點輸出，供除錯使用 |

JSON 中的 `output_dir` 是容器內路徑，例如 `/workspace/maps/floor_ground`，對應主機專案下的 `maps/floor_ground`。

每次建圖都有自己的 LIO／PGO 座標系。`session.json` 的初始位姿是後續配準的參考資料，**不是已完成優化的 map-to-world 變換**。地下樓層在模擬中的高度為 `-3.3 m`，不代表其 PCD 地板也在 `z = -3.3 m`。

要轉成 2D 地圖時，先依實際 PCD 的地板高度決定 `--z-min`／`--z-max`。以下 `0.1`～`2.0` 僅為參數範例，不能直接假設適合每個 session：

```bash
./scripts/pcd2pgm.py maps/floor_ground/map.pcd maps/floor_ground/map_2d \
  --z-min 0.1 --z-max 2.0 --resolution 0.05
```

管理器不會自動配準／合併跨樓層地圖，也不會自動切換 Nav2 地圖。

## 7. 常見問題

| 情況 | 處理方式 |
|---|---|
| ROS container 未啟動或無法連線管理器 | 先執行 `run_slam.sh`，檢查第一個終端的啟動輸出。 |
| 初始化超過 30 秒、沒有 LIO 或 ground truth | 保持模擬播放與車子停穩；查看 `slam.log`。失敗的建圖程序會停止，目錄保留；修正後用新名稱重試。 |
| RViz 無點雲 | 先確認 session 為 `mapping`；`idle` 或 `start_failed` 沒有持續建圖輸出。確認 `/fastlio2/world_cloud` 有資料、Fixed Frame 為 `lidar`，並用 `docker compose exec ros /workspace/docker/entrypoint.sh ros2 param get /rviz use_sim_time` 確認為 `true`；舊的 RViz 可用 `docker compose exec ros /workspace/docker/entrypoint.sh ros2 param set /rviz use_sim_time true` 即時修正。 |
| 樓層目錄已存在 | 使用新名稱，如 `floor_lower_run2`；不要期待從舊圖接續。 |
| 已有其他建圖節點 | 先停止另一個 launcher，不要同時使用兩套建圖流程。 |
| 存圖失敗、空圖或檔案不完整 | 不要換樓層或關閉模擬；查 `status`／`slam.log`，確認有有效資料及可寫入空間後重試 `save-stop`。 |
| 存圖成功但停止／metadata 寫入失敗 | 排除回覆中的錯誤後重試 `save-stop`；已記錄成功存圖的 session 不會再次存圖。 |
| 建圖程序意外退出 | `status` 會回報 `error`，查看該 session 的 `slam.log`。不要把意外退出當成成功存圖。 |
| 換層後 RViz 還看到上一層點雲 | 顯示快取可能保留最多 10 秒的舊點；可在 RViz 按 Reset。這不代表新的地圖混入舊樓層。 |

`save_map.sh` 仍可用於建圖期間的額外快照，但**不會停止或切換 session**，預設寫到 `maps/office` 並可能覆寫快照。分樓層建圖請使用 `slam_session.sh save-stop`。

## 8. 正常結束

若仍有正在建圖的 session，先執行 `save-stop` 並確認成功，再到第一個終端按 Ctrl+C 關閉模擬與 container。

**Ctrl+C 不會自動存圖。** 正常關閉管理器時，未存圖的 session 會記錄為 `interrupted`；此 metadata 不是已完成存圖的證明，也不能用它恢復先前的估計器。

架構、存圖實作與驗證方式見[建圖資料流程](mapping_flow.md#分樓層建圖管理器)。
