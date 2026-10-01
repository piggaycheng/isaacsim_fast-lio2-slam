# Nav2 導航架構

本文件說明 `./run_nav.sh --mode 3d --navigate` 的導航流程：costmap、路徑規劃、控制、recovery 與 `cmd_vel` 安全鏈。定位如何產生 `map -> odom` 和 `odom -> base_link` 見 [`3d_localization.md`](3d_localization.md)。

- 只有 3D 模式有導航；`run_nav.sh --mode 2d` 只做 AMCL 定位，不啟動 Nav2。
- 不加 `--navigate` 時，只以 `costmap_observer` 啟動兩張 costmap 供 RViz 觀察，不啟動 planner、controller，也不發布行駛命令。
- 設定檔都在 `ros2_ws/src/isaac_localization_3d/config/`，啟動檔是 `launch/global_fusion.launch.py`。

## 整體資料流

分成兩部分：感測資料如何進入 costmap，以及目標如何變成車子的速度命令。

### 感測 → costmap

```mermaid
flowchart TD
    Lidar["3D LiDAR<br/>/isaac/lidar_points"]
    MapServer["map_server<br/>PGM → /map"]
    subgraph Filter ["ground_obstacle_filter"]
        SelfFilter["車身形狀自體濾除<br/>保留車身外的近距離點"]
        GroundFilter["地面濾除，離地 0.06–2.0 m<br/>/perception/obstacles<br/>實線：只標記 marking"]
    end
    ScanProj["pointcloud_to_laserscan<br/>高度 0.1–2.0 m 切片<br/>/scan<br/>虛線：只清除 clearing"]

    subgraph GlobalCostmap ["global_costmap（map，2 Hz，整張地圖）"]
        GStatic["static_layer"]
        GObstacle["obstacle_layer"]
        GInflation["inflation_layer"]
    end

    subgraph LocalCostmap ["local_costmap（odom，5 Hz，8 × 8 m）"]
        LObstacle["obstacle_layer"]
        LInflation["inflation_layer"]
    end

    Lidar --> SelfFilter
    SelfFilter --> GroundFilter
    SelfFilter -->|"/perception/self_filtered_points"| ScanProj
    MapServer --> GStatic
    GroundFilter --> GObstacle
    GroundFilter --> LObstacle
    ScanProj -.-> GObstacle
    ScanProj -.-> LObstacle
```

兩張 costmap 都用 3D 定位發布的 TF 查車體位置：`map -> odom` 由 `global_tf_gate` 發布，`odom -> base_link` 由 `local_ekf` 發布（見 [`3d_localization.md`](3d_localization.md)）。

啟用 `--obstacle-cloud`（`--costmaps` / `--navigate` 會自動啟用）時，`ground_obstacle_filter` 先將點轉到 `base_link`，排除車身矩形內的點，再分兩路輸出：保留地面的 `/perception/self_filtered_points` 供 `/scan` 投影；地面擬合後的 `/perception/obstacles` 供障礙物標記。地面擬合失敗時仍會發布前者；TF 失敗則兩者都停止發布，交由 watchdog 停車。未啟用障礙點雲的 3D 定位展示與 2D AMCL 模式保留原本的原始點雲投影，不啟動這條近距離保護鏈。

### 目標 → 速度命令

```mermaid
flowchart TD
    Goal["RViz 2D Goal Pose<br/>/goal_pose"]
    BT["bt_navigator<br/>navigate_to_pose.xml"]
    Planner["planner_server<br/>NavFn + global_costmap"]
    Controller["controller_server<br/>GoalHeadingLatchedRPP + local_costmap"]
    Behavior["behavior_server<br/>BackUp、Wait"]
    Smoother["velocity_smoother<br/>加減速限制"]
    Monitor["collision_monitor<br/>前方至 x 1.30 m、周圍 y ±0.75 m<br/>停車/減速、footprint 碰撞預估"]
    Obstacles["/perception/obstacles、/scan"]
    Safety["cmd_vel_safety<br/>感測/校正過期立即停車、限速<br/>停車後從零加速"]
    Correction["校正心跳<br/>/localization_3d/accepted_correction"]
    EStop["/navigation/emergency_stop"]
    Isaac["Isaac Sim Carter<br/>DifferentialController"]

    Goal --> BT
    BT -->|"ComputePathToPose，每秒一次"| Planner
    Planner -->|"path"| BT
    BT -->|"FollowPath"| Controller
    BT -->|"失敗時 recovery"| Behavior
    Controller -->|"/nav2/cmd_vel_nav"| Smoother
    Behavior -->|"/nav2/cmd_vel_nav"| Smoother
    Smoother -->|"/nav2/cmd_vel"| Monitor
    Obstacles --> Monitor
    Obstacles -->|"資料時效 watchdog"| Safety
    Monitor -->|"/nav2/cmd_vel_monitored"| Safety
    Correction --> Safety
    EStop --> Safety
    Safety -->|"/cmd_vel"| Isaac
```

`planner_server` 和 `controller_server` 各自內含一張 costmap；`behavior_server` 則訂閱 `local_costmap` 做 BackUp 碰撞檢查。recovery 時 BT 也會呼叫兩張 costmap 的清除服務，見下方「Behavior tree 與 recovery」。

## 兩張 costmap

costmap 是 2D 格子地圖，每格 0.05 m，存 0–255 的代價：0 是空的、254 是障礙物（lethal）、255 是未知，中間是 inflation 擴散的代價。各圖層分別寫入後合併。設定在 `observation_costmaps.yaml`。

| | `global_costmap` | `local_costmap` |
| :-- | :-- | :-- |
| 使用者 | `planner_server`：規劃整條路徑 | `controller_server`：沿路徑決定速度；`behavior_server`：BackUp 碰撞檢查 |
| 座標系 | `map` | `odom` |
| 範圍 | 整張 PGM | 以車為中心 8 × 8 m 滾動視窗 |
| 更新頻率 | 2 Hz | 5 Hz |
| 圖層 | `static_layer`、`obstacle_layer`、`inflation_layer` | `obstacle_layer`、`inflation_layer` |

分成兩張是因為用途不同：

- **global 管「走哪條路」**，需要完整：包含 PGM 牆壁，並記得看過的障礙物，直到被清除。
- **local 管「現在怎麼走」**，需要快而穩：它放在 `odom`，不會因 PCD 校正讓 `map -> odom` 跳動而跟著跳，controller 不會因此急轉或急停；它不載入 PGM，只看感測器，定位誤差時不會被對不準的地圖牆壁擋住。

兩張設定相同的 `base_link` 矩形 footprint（前 0.65 m、後 0.20 m、左右各 0.32 m），並設定 `footprint_padding: 0.01` m。footprint 是車體在地面的占用輪廓，不是煞停區；目前涵蓋 Nova Carter 的車體幾何並保留裕度。0.9 m inflation 半徑用於產生導航代價，不是安全煞停距離。

### 障礙物標記與清除

兩張 costmap 的 `obstacle_layer` 都有兩個輸入，分工不同：

| 輸入 | 用途 | 設定 |
| :-- | :-- | :-- |
| `/perception/obstacles`（3D，已濾除地面） | 只標記 | `marking: true`、`clearing: false`、`obstacle_max_range: 6.0` |
| `/scan`（2D 切片） | 只清除 | `marking: false`、`clearing: true`、`raytrace_max_range: 7.0` |

- **用 3D 標記**：可以看到桌面、懸空物等 2D 切片可能掃不到的障礙物，且地面濾除比單純高度切片更不容易把地面誤判成障礙物。
- **用 2D 清除**：清除靠 raytrace，把感測器到打到點之間的格子設成空的。`ObstacleLayer` 是 2D 的，若用 3D 點雲清除，越過矮障礙物、打到後方牆壁的射線壓平後會穿過障礙物所在的格子，把剛標記的障礙物清掉。`/scan` 每個角度只取最近的點，射線停在第一個障礙物上，不會誤清。3D 點雲的 raytrace 計算量也遠大於 720 條 scan 射線。
- 要用 3D 清除，需改用 3D 體素圖層，例如 `VoxelLayer` 或 STVL（規劃項目）。

costmap 標記過的格子不會自己消失，只能靠清除：

- 車子看得到原位置時，障礙物移走後 `/scan` 會在 1 秒內清掉它；BT 每秒重新規劃，路徑約 1–2 秒內改回。
- 在車子背後、超過 7 m、被擋住，或低於 `/scan` 的 0.1 m 下限時，清除不到，會留下殘影。需等車子重新看到，或 recovery 清除整張 costmap。
- `/scan` 設了 `use_inf: true`，但 costmap 未開 `inf_is_valid`，沒有打到東西的方向不會被清除。

## 規劃、控制與避障

| 元件 | 設定 | 說明 |
| :-- | :-- | :-- |
| Planner | `nav2_navfn_planner/NavfnPlanner`（Dijkstra），`allow_unknown: false` | 在 global costmap 找最低代價路徑 |
| Controller | `isaac_nav::GoalHeadingLatchedRPP`，10 Hz | Regulated Pure Pursuit：目標速度 0.5 m/s、前視距離 0.8 m；依曲率、接近目標與碰撞預測降速；到達位置後鎖定原地轉向，不會因 1 Hz 重新規劃而中斷 |
| Goal checker | `isaac_nav::LatchedGoalChecker` | 位置誤差 0.15 m、航向 0.25 rad；到位後鎖定，偏離超過 0.5 m 才解除 |
| Progress checker | `SimpleProgressChecker` | 15 秒內移動不到 0.15 m 視為卡住 |
| `failure_tolerance` | 1.0 秒 | controller 持續失敗超過 1 秒就回報失敗，交給 BT 處理 |

**繞過障礙物靠 global costmap 重新規劃，不是靠 controller。** RPP 只會沿著路徑走；local costmap 對它的作用只有碰撞預測時停車與降速。障礙物進入 global costmap 後，BT 每秒呼叫 planner，新路徑就會繞開。

會在 local costmap 內自行繞開的是 MPPI、DWB 這類取樣式 controller，但它們只看未來幾秒、8 m 視窗內的範圍。整條路被擋時仍需 global 重新規劃，所以兩者是互補的：global 決定路線；local controller 應付移動中的障礙與小幅偏移。目前尚未換用 MPPI。

測試時可用 `--box` 在 Isaac 場景放一個靜態箱子。箱子不在 PGM/PCD 地圖裡，只靠 LiDAR 進入 costmap：

```bash
./run_nav.sh --mode 3d --navigate --box 2.0,0.0
```

從 RViz 送 (3.5, 0) 的目標，路徑會繞過箱子。

### 遇到動態障礙物時的處理順序

下圖是行駛中前方突然出現障礙物（例如行人走進路線）時，各元件的處理順序。**減速與停車**由 `collision_monitor` 和 RPP 分工處理，**繞過障礙物**只靠 global costmap 重新規劃。圖中的時間是依更新頻率推算的上限，不是實測值。

```mermaid
sequenceDiagram
    autonumber
    participant L as LiDAR<br/>/perception/obstacles、/scan
    participant CM as collision_monitor
    participant LC as local_costmap<br/>5 Hz
    participant RPP as controller_server<br/>RPP 10 Hz
    participant VS as velocity_smoother<br/>20 Hz
    participant GC as global_costmap<br/>2 Hz
    participant BT as bt_navigator<br/>重新規劃 1 Hz
    participant P as planner_server
    participant S as cmd_vel_safety → Isaac

    Note over L,S: 障礙物出現在車前方
    L->>CM: 最新點雲與 scan（每幀）
    RPP->>VS: /nav2/cmd_vel_nav
    VS->>CM: /nav2/cmd_vel（加減速限制後）
    CM->>S: 進入減速區 → 50%；碰撞預估 → 按比例降速；進入停車區 → 0
    Note over S: 停車立即歸零；解除後從零逐步加速
    Note over CM: 第一道反應，不需等 costmap 更新

    L->>LC: 標記障礙物（≤ 0.2 s）
    LC->>RPP: 路徑附近代價升高
    RPP->>VS: 依代價降速（Humble 預設 use_cost_regulated_linear_velocity_scaling）
    VS->>CM: 平滑後命令
    alt 1 s 內（到前視點為止）會撞上
        RPP->>VS: 發布零速
        VS->>CM: 一般控制停車依減速度降速
        Note over RPP: 持續超過 failure_tolerance 1 s<br/>→ FollowPath 失敗，清除 local costmap 後重試
    end

    L->>GC: 標記障礙物（≤ 0.5 s）
    BT->>P: ComputePathToPose（每秒一次）
    P-->>BT: 繞開障礙物的新路徑
    BT->>RPP: FollowPath（新路徑）
    RPP->>CM: 沿新路徑恢復速度
    CM->>S: 障礙物離開各區域後原速放行

    opt 內層重試後仍失敗（規劃失敗，或 RPP 持續停車、15 s 無進展）
        BT->>BT: recovery 輪流執行：清除 costmap → Wait 5 s → BackUp 0.3 m → Wait 10 s<br/>最多 6 次（約 30 s）後中止目標
    end

    Note over L,GC: 障礙物移走後，/scan 在看得到的範圍內清除 costmap，<br/>之後的重新規劃會改回原本較短的路徑
```

| 階段 | 誰負責 | 依據的資料 | 動作 |
| :-- | :-- | :-- | :-- |
| 緊急反應 | `collision_monitor` | 最新一幀感測資料 | 減速、按比例降速或停車；不會換路線 |
| 平順減速 | RPP | local costmap | 靠近高代價區時降速；預估 1 s 內會撞就停車並回報失敗 |
| 繞過障礙物 | `bt_navigator` + `planner_server` | global costmap | 每秒重新規劃，產生繞開的路徑 |
| 繞不過去或卡住 | `bt_navigator` + `behavior_server` | — | 規劃或行駛任一方在內層重試後仍失敗時，清除 costmap、等待、後退，仍失敗就中止 |

RPP 不會在 local costmap 裡自己找路繞開，只會沿著 planner 給的路徑減速或停車。因此移動中的障礙物如果還沒進入 global costmap，車子只會減速或停下等待，不會閃避。

## Behavior tree 與 recovery

`navigate_to_pose.xml` 和 `navigate_through_poses.xml` 結構相同。`bt_navigator` 只在啟動時讀取 XML，修改後要重啟 stack。

```mermaid
flowchart TD
    Root["RecoveryNode NavigateRecovery<br/>最多重試 6 次"]
    Root --> Pipeline["PipelineSequence<br/>NavigateWithReplanning"]
    Root --> Fallback["ReactiveFallback<br/>RecoveryFallback"]

    Pipeline --> Rate["RateController 1 Hz"]
    Rate --> PlanRec["RecoveryNode ComputePath（重試 1 次）"]
    PlanRec --> Plan["ComputePathToPose"]
    PlanRec --> ClearG1["ClearEntireCostmap（global）"]
    Pipeline --> FollowRec["RecoveryNode FollowPath（重試 1 次）"]
    FollowRec --> Follow["FollowPath"]
    FollowRec --> ClearL1["ClearEntireCostmap（local）"]

    Fallback --> GoalUpdated["GoalUpdated<br/>有新目標就中止 recovery"]
    Fallback --> RR["RoundRobin：每次失敗輪到下一個"]
    RR --> Clear["1. 清除 local + global costmap"]
    RR --> Wait5["2. Wait 5 秒"]
    RR --> Backup["3. BackUp 0.3 m，0.1 m/s"]
    RR --> Wait10["4. Wait 10 秒"]
```

1. **內層**：planner 或 controller 失敗時，先清除自己的 costmap，再重試一次。
2. **外層**：仍失敗時，依序輪流執行一個 recovery 動作，再從頭規劃與行駛。最多 6 次，約 30 秒，讓車子有時間等其他車輛或行人通過。之後才中止目標。
3. 收到新目標（`GoalUpdated`）時，立即中止 recovery。

BackUp 和 Wait 是 `nav2_behaviors` 內建的 plugin，由 `behavior_server` 執行（`navigation.yaml` 的 `behavior_plugins: ["backup", "wait"]`）。BackUp 以 local costmap footprint 模擬 2 秒內的後退路徑，會撞到就不執行。

ClearEntireCostmap 會清掉 global costmap 所有看過的障礙物，但 static layer 會從 `/map` 還原。清除的目的是去掉殘影、定位跳動或誤判造成的假障礙物；真的障礙物會在 1 秒內被 LiDAR 重新標記。

## `cmd_vel` 安全鏈

`controller_server` 和 `behavior_server` 都不直接控制車子，它們的 `/cmd_vel` 被重新映射到 `/nav2/cmd_vel_nav`，先經過 `velocity_smoother` 限制加減速，再依序經過兩道安全關卡才到 Isaac Sim 訂閱的 `/cmd_vel`：

```
controller / behavior → /nav2/cmd_vel_nav → velocity_smoother → /nav2/cmd_vel → collision_monitor → /nav2/cmd_vel_monitored → cmd_vel_safety → /cmd_vel
```

兩道安全關卡都只會降速或停車，不會加速，所以最終速度取最嚴格的限制。smoother 放在安全關卡之前，安全停車不會被平滑延遲。

### velocity_smoother

Nav2 Humble 內建的 `nav2_velocity_smoother`，設定在 `navigation.yaml`。RPP 與 BackUp 的命令可能一步從 0 跳到 0.5 m/s，smoother 以 20 Hz 把它變成斜坡。

| 參數 | 值 | 說明 |
| :-- | :-- | :-- |
| `max_accel` | 線 0.8 m/s²、角 1.5 rad/s² | 0 → 0.5 m/s 約 0.6 秒 |
| `max_decel` | 線 −1.5 m/s²、角 −2.0 rad/s² | 正常減速；安全停車不受此限制 |
| `max_velocity` / `min_velocity` | ±0.75 m/s、±0.7 rad/s | 與 `cmd_vel_safety` 的限速一致 |
| `velocity_timeout` | 0.5 s | Nav2 停止發布命令後，依減速度降到 0，然後停止發布 |
| `feedback` | `OPEN_LOOP` | 以上一個輸出作為目前速度 |

- 不用 `CLOSED_LOOP`：它以量測速度加一步加速度作為輸出，原地旋轉時 Carter 的輪子在約 0.1 rad/s 時克服不了摩擦，量測速度一直是 0，車子會卡住不轉。
- `OPEN_LOOP` 看不到下游的停車，因此由最後一道 `cmd_vel_safety` 記錄實際發布的命令；即使 smoother 在停車期間仍送原速，解除後最終命令也會從零逐步加速。
- smoother 的計時器使用 wall time，模擬比實際時間慢時，以模擬時間換算的加速度會略高。

### collision_monitor

Nav2 Humble 內建的 `nav2_collision_monitor`，設定在 `collision_monitor.yaml`。它不看 costmap，每收到一次速度命令，就用最新的感測資料檢查一次，不會有 costmap 的更新延遲和殘影。RPP 負責依 costmap 平順降速與停車，`collision_monitor` 則是最後一道防撞保護，應付 costmap 還沒更新的突發近距離障礙物。

| 區域 | 範圍（`base_link`） | 動作 |
| :-- | :-- | :-- |
| `PolygonStop` | 車頭前 0.65 m（x 0.65–1.30 m，y ±0.36 m） | 超過 3 個點就停車（線速度、角速度都歸零） |
| `PolygonSurround` | x −0.80–1.10 m，y ±0.75 m；相對未 padding 的 footprint，前擴 0.45 m、後擴 0.60 m、側擴 0.43 m | 超過 3 個點就停車，涵蓋側面、後方與近車頭 |
| `PolygonSlow` | 車頭前 0.75 m（x 0.65–1.40 m，y ±0.50 m） | 超過 3 個點就降為 50% |
| `FootprintApproach` | local costmap 的 footprint（`/local_costmap/published_footprint`） | 沿目前命令模擬 1.5 秒，依距離碰撞的時間按比例降速；會考慮行進方向，後退和原地旋轉也會檢查 |

固定停車區需在車體輪廓外預留感測與控制延遲、物理煞停行程及安全裕度；發布零速不代表車體瞬間停止。`FootprintApproach` 使用 local costmap 發布的 footprint，但固定停車／減速區不會隨 footprint 自動更新。換車或提高速度時需重新調整並驗證，不能只修改 footprint。

- 感測來源：`/perception/obstacles`（地面濾除後的 3D 點）和 `/scan`（2D 切片）。找不到地面時，`ground_obstacle_filter` 會停止發布，這時仍有 `/scan` 可用。
- 不再排除半徑 0.5 m 內的所有點。`ground_obstacle_filter.yaml` 的 `self_filter_bounds: [-0.20, 0.65, -0.32, 0.32]` 只排除車身矩形內的點（向上延伸，避免自體反射）；`min_range: 0.0`。導航的 `/scan` 使用同一份自體濾除點雲，並覆寫 `range_min: 0.0`、`range_max: 20.0`，所以車身外的近距離點可以同時進入 scan 與障礙點雲。
- `PolygonSurround` 不分行進方向：側方或後方太近時，前進、倒退、原地旋轉都會被擋下。障礙物離開且新資料確認區域淨空後才恢復。
- 這解決的是軟體距離濾除造成的盲區，不代表 LiDAR 沒有物理遮蔽、量測最短距離或點數不足的盲區。車身自體濾除範圍與保護區必須隨機器人幾何一起調整，必要時需加近距離感測器。
- 停車區不分方向：障礙物在停車區內時，BackUp 後退也會被擋下，只能等障礙物離開，或在 recovery 用完後中止目標。
- 來源時間戳落後目前 ROS 時間達 1 秒時，Humble 版會忽略該來源；最後一層 `cmd_vel_safety` 會在兩個來源都過期或尚未收到時強制停車。
- 停車後會繼續發布零速 2 秒（`stop_pub_timeout`），之後停止發布；Isaac Sim 在 0.5 秒收不到命令時也會自行停車。
- RViz 會顯示前方停車區（紅）、周圍停車區（粉紅）和減速區（橘）。

### cmd_vel_safety

| 條件 | 行為 |
| :-- | :-- |
| 命令含 NaN/Inf，或有非平面分量（`linear.y/z`、`angular.x/y`） | 發布零速 |
| 4 秒內沒有 `/localization_3d/accepted_correction`（PCD 校正過期） | 發布零速 |
| `/scan` 與 `/perception/obstacles` 都沒有 1 秒內的新資料 | 發布零速；任一來源恢復後才允許新的速度命令 |
| 收到 `/navigation/emergency_stop` 為 `true` | 鎖定停車，需重啟才能恢復 |
| 0.5 秒未收到新的速度命令 | 發布零速；下一個命令從零起步 |
| 其他 | 限速 0.75 m/s、0.7 rad/s，再限制加速；減速與停車立即轉發 |

最後一道關卡只限制速度大小增加：`max_linear_accel: 0.8` m/s²、`max_angular_accel: 1.5` rad/s²（`collision_monitor.yaml`，與 smoother 的 `max_accel` 一致）。collision monitor 發布零速、感測或定位校正過期、無效命令與 emergency stop 都立即歸零並重設起步狀態；安全停車不受減速度限制。恢復後必須收到新命令，從最後發布的零速逐步增加；倒退與原地旋轉也適用。方向反轉先輸出零速，再向相反方向加速。

加速使用 ROS 時間與上次輸出間隔，每次最多計入 0.1 秒，避免停車或延遲期間累積加速額度。第一個命令輸出零速，之後隨新命令逐步增加；不在 timer 內重播命令。`command_timeout: 0.5` 秒與 Isaac Sim 的命令逾時一致，命令中斷時由 watchdog 停車，且即使 timer 尚未執行，逾時後的第一個命令也會重新從零開始。時間倒退同樣重設起步狀態。

安全節點不會放寬校正時效；`global_tf_gate` 也會在校正過期時停止發布 `map -> odom`，Nav2 因此查不到 TF，無法繼續規劃與控制。

感測 watchdog 每 0.1 秒檢查一次，即使 monitor 沒有再送速度命令，也會主動發布零速；不會在資料恢復後重播舊命令。`sensor_timeout` 設定在 `collision_monitor.yaml`，需與 monitor 的 `source_timeout` 保持一致。訂閱使用 sensor-data QoS，可接收 best-effort 感測資料。新鮮度依訊息時間戳及 ROS 時間計算：模擬暫停且 `/clock` 停止時不會按現實時間過期；時間倒退時清空感測與定位校正狀態，需重新收到資料。這只檢查資料時效，不能判斷感測盲區或資料是否完整可用。

## 啟動順序

```mermaid
flowchart LR
    Start["launch 啟動定位、map_server<br/>與 Nav2 節點（未啟用）"] --> Ready["wait_for_costmap_tf.py<br/>等待 /map 與 map -> base_link"]
    Ready -->|"--navigate"| NavMgr["lifecycle_manager_navigation<br/>planner_server、controller_server、<br/>behavior_server、bt_navigator、<br/>velocity_smoother、collision_monitor"]
    Ready -->|"未加 --navigate"| CostMgr["lifecycle_manager_fusion_costmaps<br/>只啟用兩個 costmap_observer"]
    NavMgr --> NavReady["wait_for_navigation.py<br/>確認六個節點都 active"]
    NavReady -->|"成功"| Log["log：navigator active"]
    Ready -->|"失敗"| Stop["關閉整個 launch"]
    NavReady -->|"失敗"| Stop
```

以預設 Office 地圖啟動時，`global_pose_adapter` 會自動發送 Carter 出生點附近的初始位姿，不需要在 RViz 點選。log 出現 `navigator active` 後，即可用 RViz「2D Goal Pose」送目標。

## 目前限制

- `collision_monitor` 的區域已通過 Office／Nova Carter 的平地模擬煞停驗證；點數門檻、稀疏／低矮障礙及更差的感測延遲仍未驗證，不能當成認證安全區。
- RPP 不會在 local costmap 內主動繞開移動中的障礙物。
- 尚未使用 costmap filters（Keepout 禁行區、Speed 限速區、Binary 開關區）。目前無法在地圖上劃出禁止進入或限速的區域，只能靠修改 PGM 地圖或 inflation 來間接達成。
- footprint 已量測並涵蓋 Carter 幾何；inflation、斜坡、動態障礙物清除與狹窄路線的碰撞安全仍未驗證。固定 `PolygonSurround` 全寬 1.50 m，可能擋住車身本來能通過的窄門。
- topic 與 TF frame 都是固定名稱，還不支援多台機器人（namespace）。
- 真實車輛導航安全尚未驗證。Office 模擬已包含實體箱子煞停與感測故障注入；使用時仍先在 RViz 確認 costmap 與規劃路徑。
