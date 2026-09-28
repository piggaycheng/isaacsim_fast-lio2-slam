# 2D 定位現況與 3D 定位目標架構

目前 `run_nav.sh` 僅實作 2D 定位：輪式里程計與 IMU 進 Local EKF，
由 Local EKF 發布 `odom -> base_link`；3D LiDAR 投影成 `/scan` 供 AMCL，
由 AMCL **獨自**發布 `map -> odom`。沒有啟動 Global EKF 或 3D 定位。
停車時 AMCL 若沒有新位姿，`map -> odom` 會保持不變，避免 Global EKF
在缺少全域校正時繼續預測出不合理的位移。

未來若提供與 PGM 同座標系的 PCD，規劃切換到以下目標架構：
停用 AMCL（PGM 只供 Nav2 costmap 使用），將
[FAST_LIO_LOCALIZATION2](https://github.com/Smart-Wheelchair-RRC/FAST_LIO_LOCALIZATION2)
的 PCD 配準結果轉為品質閘控後的全域 pose，結合輪式里程計與 IMU，
由全域融合節點獨自發布 `map -> odom`。**兩種模式不能同時發布這條 TF。**
獨立的 3D 定位模式已由 `isaac_localization_3d` 套件及 `./run_3d_localization.sh`
提供，可在 RViz 的 Office PGM 地圖上觀察 PCD 配準位置。此模式的
`map -> camera_init -> body -> base_link` TF 只在配準被接受後發布
`map -> camera_init`，**不與**現行 2D 導航同時啟動；它不代表以下
PGM + PCD 導航全域融合已實作。操作方式見
[`ros2_ws/README.md`](../ros2_ws/README.md)。

## 只有 PGM：純 2D 定位（目前已實作）

以下是 `run_nav.sh` 的定位資料流；PGM 同時可供後續 Nav2 的
global costmap 使用。此模式不載入 PCD、不啟動 FAST_LIO_LOCALIZATION2
或 Global EKF。

```mermaid
flowchart TD
    subgraph Sensors ["Isaac Sim 感測器"]
        Joints["左右輪關節角度"]
        IMU["IMU"]
        Lidar["3D LiDAR<br/>PointCloud2"]
    end

    Joints --> WheelOdom["輪式里程計<br/>/wheel/odom"]
    IMU --> IMUAdapter["IMU adapter<br/>/nav/imu"]
    WheelOdom --> LocalEKF["Local robot_localization<br/>world_frame: odom"]
    IMUAdapter --> LocalEKF
    LocalEKF -->|"唯一 TF: odom -> base_link"| AMCL["AMCL"]

    Lidar --> Scan["pointcloud_to_laserscan<br/>/scan"]
    Scan --> AMCL
    PGM["PGM 地圖<br/>map.yaml + map.pgm"] --> MapServer["Nav2 map_server"]
    MapServer --> AMCL
    AMCL -->|"唯一 TF: map -> odom"| RViz["RViz / 車體全域位置"]
    LocalEKF --> RViz
    MapServer --> RViz
```

## PGM + PCD 模式（全域融合測試已實作；Nav2 尚未整合）

```mermaid
flowchart TD
    %% -------------------- 感測與底盤 --------------------
    subgraph Sensing ["感測與底盤 Sensing & Base"]
        WheelJoints["左右輪關節角度"]
        WheelOdom["輪式里程計<br/>編碼器量化、偏差與雜訊模型"]
        IMU["IMU"]
        Lidar3D["單一 3D LiDAR<br/>PointCloud2"]
        Chassis["底盤驅動器"]
    end

    %% -------------------- 點雲前處理 --------------------
    subgraph LidarPreprocessing ["3D LiDAR 前處理 LiDAR Preprocessing"]
        Deskew["時間同步、deskew 與 TF 轉換"]
        ScanProjection["高度裁切 + pointcloud_to_laserscan<br/>虛擬 2D LaserScan"]
    end

    %% -------------------- 局部狀態估計 --------------------
    subgraph LocalEstimation ["局部狀態估計 Local Estimation"]
        LocalEKF["Local robot_localization<br/>world_frame: odom"]
    end

    %% -------------------- 3D 全域定位 --------------------
    subgraph Localization3D ["3D 全域定位 3D Localization（不啟動 AMCL）"]
        PCD["3D 地圖<br/>map.pcd"]
        FastLIO["FAST-LIO 里程計<br/>LiDAR + IMU；不接管導航 TF"]
        FastLIOLocalization["FAST_LIO_LOCALIZATION2<br/>既有 PCD 地圖 ICP 配準"]
        FastLIOAdapter["3D 定位 Adapter<br/>位姿組合、掃描／里程計新鮮度與跳動檢查<br/>使用上游內建 fitness 門檻；設定 covariance"]
    end

    %% -------------------- 2D 導航地圖 --------------------
    PGM["PGM 地圖<br/>只供 Nav2 costmap；不供 AMCL 定位"]

    %% -------------------- 全域融合 --------------------
    subgraph GlobalFusion ["全域融合 Global Fusion"]
        GlobalEKF["Global robot_localization<br/>world_frame: map"]
        TFGate["校正時效閘控<br/>唯一發布 map → odom"]
    end

    %% -------------------- 3D 障礙物處理 --------------------
    subgraph Perception3D ["3D 障礙物處理 3D Perception"]
        GroundFilter["地面濾除<br/>例如 Patchwork++"]
        ObstacleProjection["障礙物投影 / 體素化<br/>pointcloud_to_laserscan / STVL"]
    end

    %% -------------------- Nav2 --------------------
    subgraph Nav2Stack ["Nav2 Navigation Stack"]
        BTNav["BT Navigator"]
        GlobalCostmap["Global Costmap"]
        LocalCostmap["Local Costmap"]
        GlobalPlanner["Global Planner"]
        LocalController["Local Controller"]
    end

    %% 輪式里程計與局部 EKF
    WheelJoints --> WheelOdom
    WheelOdom --> LocalEKF
    IMU --> LocalEKF
    WheelOdom --> GlobalEKF
    IMU --> GlobalEKF
    LocalEKF -->|"唯一 TF: odom -> base_link"| LocalCostmap
    LocalEKF -->|"唯一 TF: odom -> base_link"| LocalController

    %% PCD 配準提供唯一的全域定位觀測
    PCD --> FastLIOLocalization
    Lidar3D --> Deskew
    Lidar3D -.->|"含逐點時間的原始掃描"| FastLIO
    IMU -.-> FastLIO
    FastLIO -.->|"局部里程計 + 配準點雲"| FastLIOLocalization
    InitialPose["map 座標初始位姿<br/>手動指定 / Office 起點近似先驗"] --> FastLIOLocalization
    FastLIOLocalization -.->|"map 到 LIO 起點的配準結果<br/>需擴充 fitness 輸出"| FastLIOAdapter
    FastLIO -.->|"LIO 里程計"| FastLIOAdapter
    FastLIOAdapter -.->|"map 座標 pose"| GlobalEKF

    %% Global EKF 不直接發布 TF；閘控節點阻止過期校正時的 TF 更新
    FastLIOAdapter -.->|"校正心跳"| TFGate
    GlobalEKF -->|"/odometry/global；不發布 TF"| TFGate
    TFGate -->|"唯一 TF: map -> odom"| GlobalCostmap
    TFGate -->|"唯一 TF: map -> odom"| GlobalPlanner

    %% Nav2 地圖與即時障礙物
    PGM -->|"靜態 occupancy grid"| GlobalCostmap
    Deskew --> ScanProjection
    ScanProjection --> LocalCostmap
    Deskew --> GroundFilter
    GroundFilter --> ObstacleProjection
    ObstacleProjection --> LocalCostmap

    %% Nav2 資料流
    BTNav --> GlobalPlanner
    BTNav --> LocalController
    GlobalCostmap --> GlobalPlanner
    GlobalPlanner -->|"Path"| LocalController
    LocalCostmap --> LocalController
    LocalController -->|"cmd_vel"| Chassis
```

## TF 發布權責

| TF | 唯一發布者 |
|---|---|
| `map -> odom` | 只有 PGM：AMCL；PCD 全域融合測試：校正時效閘控節點（Global EKF 不發布 TF，不啟動 AMCL） |
| `odom -> base_link` | Local `robot_localization` |
| `base_link -> lidar`、`base_link -> imu` | `robot_state_publisher` 或 static TF |

目前 `run_nav.sh` 的 AMCL 使用 `tf_broadcast: true`。規劃中的 PCD
融合測試模式不啟動 AMCL，由校正時效閘控節點接管 `map -> odom`。
`FAST_LIO_LOCALIZATION2` 原版
`transform_fusion.py` 會發布 `map -> camera_init` TF，不能不修改就與目前
`odom -> base_link` 的導航 TF 鏈並用；規劃模式不啟動原版 TF 發布路徑，
由 Adapter 組合 `/map_to_odom` 配準結果與 `/Odometry` LIO 里程計，
轉成 `map` 座標的 `base_link` pose，交給 Global EKF 作為觀測。
原版 fitness 只供內部閾值判斷和日誌使用；要讓 Adapter
依分數設定可信 covariance，仍須擴充上游品質輸出。
目前已新增**隔離測試用** `./run_3d_localization.sh --global-fusion`：
3D 位姿 Adapter 使用上游內建的 ICP fitness 門檻，以及掃描／里程計
新鮮度、位姿跳動檢查，將 `map` 座標的 2D 車體位姿送入 Global EKF；
輪速及 IMU 同時供 Local／Global EKF 預測。為避免缺少全域校正時
Global EKF 不斷發布漂移 TF，Global EKF 設為 `publish_tf: false`，
另由閘控節點在收到近期校正時發布**唯一**的 `map -> odom`，
Local EKF 發布 `odom -> base_link`。此模式不啟動 AMCL 或 Nav2，
不與 `run_nav.sh`／原本的獨立 3D 展示模式同時執行。
Nova Carter 驅動輪的 USD 接地碰撞體半徑為 0.14 m（輪距 0.4132 m）；
控制器與輪速里程計須使用相同幾何，否則移動時輪速低估、
`map -> odom` 必須持續補償，掃描會相對地圖漂移。
目前也沿用 2D 模式的 `pointcloud_to_laserscan`，對 `/isaac/lidar_points`
以 `base_link` 高度 0.1–2.0 m 裁切後發布 `/scan`，供 RViz 對照 PGM
檢查障礙物投影。這只是 Nav2 local costmap 的**候選輸入**，還沒有
地面分割、3D 障礙物體素化或已啟動的 Nav2 costmap；低於 0.1 m 的
障礙物可能被濾掉，須驗證使用場景與高度設定後再用於自主避障。
上游尚未輸出 ICP fitness 數值或可信 covariance；目前使用可調的保守
測量 covariance，並未完成真值精度驗證或導航失效安全驗證，
不能將本模式視為可上線的自主導航。

## 運作模式

- **只有 PGM（目前已實作）：**Local EKF 融合輪式里程計和 IMU；AMCL
  使用 `/scan` 和 PGM 定位，直接發布 `map -> odom`。
- **提供 PCD（全域融合隔離測試已新增；Nav2 尚未整合）：**不啟動
  AMCL；PGM 用於 RViz，未來供 Nav2 costmap。FAST_LIO_LOCALIZATION2
  以 LiDAR–IMU 里程計和 PCD 配準提供經 Adapter 檢查的全域位姿；
  輪速與 IMU 供局部／全域 EKF 預測，經校正時效閘控發布
  `map -> odom`。
- **3D 配準品質不佳：**不能在沒有全域觀測時無限依賴輪速／IMU 預測；
  需偵測修正逾時、限制漂移並停止或降級導航，停車時的 Global EKF
  漂移仍須先查明；不會自動退回 AMCL。
- **地面車限制：**導航主要使用 `x/y/yaw`；`roll/pitch` 以 IMU 為主，`z` 應固定
  或嚴格限制，避免平坦環境中的 3D 配準漂移。

PGM 應由同一份 PCD 投影產生，並保留一致的 `map` 原點、方向與尺度，
否則 3D 定位的車體位姿與 Nav2 的 2D costmap 無法正確對齊。
即時虛擬 LaserScan 的高度裁切範圍也應與 PGM 地圖的障礙物相符。

Local EKF 與 Global EKF 可訂閱相同的輪式里程計和 IMU 原始資料，但 Global EKF
不應直接融合 Local EKF 的完整 pose 輸出。這可避免 Global EKF 為了轉換
`odom` frame 的 pose 而依賴自己發布的 `map -> odom`，形成 TF 循環。
FAST-LIO 的局部里程計只供該 3D 定位分支使用，不再當成另一筆獨立的
Global EKF 里程計輸入。
