# 2D 定位現況與 3D 定位目標架構

目前 `run_nav.sh` 僅實作 2D 定位：輪式里程計與 IMU 進 Local EKF，
由 Local EKF 發布 `odom -> base_link`；3D LiDAR 投影成 `/scan` 供 AMCL，
由 AMCL **獨自**發布 `map -> odom`。沒有啟動 Global EKF 或 3D 定位。
停車時 AMCL 若沒有新位姿，`map -> odom` 會保持不變，避免 Global EKF
在缺少全域校正時繼續預測出不合理的位移。

未來若加入與 PGM 同座標系的 PCD，才考慮以下目標架構：
關閉 AMCL 的 TF broadcast，將
[FAST_LIO_LOCALIZATION2](https://github.com/Smart-Wheelchair-RRC/FAST_LIO_LOCALIZATION2)
的 PCD 配準結果轉為品質閘控後的全域 pose，改由全域融合節點獨自發布
`map -> odom`。**兩種模式不能同時發布這條 TF。**
獨立的 3D 定位模式已由 `isaac_localization_3d` 套件及 `./run_3d_localization.sh`
提供，可在 RViz 的 Office PGM 地圖上觀察 PCD 配準位置。此模式的
`map -> camera_init -> body -> base_link` TF 只在配準被接受後發布
`map -> camera_init`，**不與**現行 2D 導航同時啟動；它不代表以下
PGM + PCD 導航全域融合已實作。操作方式見
[`ros2_ws/README.md`](../ros2_ws/README.md)。

## 規劃中的 PGM + PCD 模式（尚未實作）

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

    %% -------------------- 2D 全域定位 --------------------
    subgraph Localization2D ["主要全域定位 Primary 2D Localization"]
        PGM["2D 地圖<br/>map.yaml + map.pgm"]
        AMCL["AMCL<br/>tf_broadcast: false"]
    end

    %% -------------------- 可選 3D 全域定位 --------------------
    subgraph Localization3D ["可選精密定位 Optional 3D Localization"]
        PCD["3D 地圖<br/>map.pcd"]
        FastLIO["FAST-LIO 里程計<br/>LiDAR + IMU；不接管導航 TF"]
        FastLIOLocalization["FAST_LIO_LOCALIZATION2<br/>既有 PCD 地圖 ICP 配準"]
        FastLIOAdapter["3D 定位品質 Adapter<br/>位姿組合、fitness、時間戳、跳動檢查<br/>設定 covariance；不發布 TF"]
    end

    %% -------------------- 全域融合 --------------------
    subgraph GlobalFusion ["全域融合 Global Fusion"]
        GlobalEKF["Global robot_localization<br/>world_frame: map"]
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

    %% AMCL 永遠作為主要全域定位
    PGM --> AMCL
    ScanProjection -->|"/scan"| AMCL
    AMCL -->|"PoseWithCovariance<br/>不發布 TF"| GlobalEKF

    %% 有 PCD 時才啟動的 FAST-LIO 全域定位分支
    PCD -.->|"有提供 PCD 才啟動"| FastLIOLocalization
    Lidar3D --> Deskew
    Lidar3D -.->|"含逐點時間的原始掃描"| FastLIO
    IMU -.-> FastLIO
    FastLIO -.->|"局部里程計 + 配準點雲"| FastLIOLocalization
    AMCL -.->|"map 座標初始位姿 / 重新定位"| FastLIOLocalization
    FastLIOLocalization -.->|"map 到 LIO 起點的配準結果<br/>需擴充 fitness 輸出"| FastLIOAdapter
    FastLIO -.->|"LIO 里程計"| FastLIOAdapter
    FastLIOAdapter -.->|"map 座標 pose"| GlobalEKF

    %% Global EKF 是 map -> odom 的唯一發布者
    GlobalEKF -->|"唯一 TF: map -> odom"| GlobalCostmap
    GlobalEKF -->|"唯一 TF: map -> odom"| GlobalPlanner

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
| `map -> odom` | 目前 2D：AMCL；規劃中 PGM + PCD：Global `robot_localization`（AMCL 關閉 TF） |
| `odom -> base_link` | Local `robot_localization` |
| `base_link -> lidar`、`base_link -> imu` | `robot_state_publisher` 或 static TF |

目前 `run_nav.sh` 的 AMCL 使用 `tf_broadcast: true`。僅在啟動 Global EKF
且要由其接管 TF 的規劃中 PGM + PCD 模式，才將 AMCL 設為
`tf_broadcast: false`。`FAST_LIO_LOCALIZATION2` 原版
`transform_fusion.py` 會發布 `map -> camera_init` TF，不能不修改就與目前
`odom -> base_link` 的導航 TF 鏈並用；規劃模式不啟動原版 TF 發布路徑，
由 Adapter 組合 `/map_to_odom` 配準結果與 `/Odometry` LIO 里程計，
轉成 `map` 座標的 `base_link` pose，僅交給 Global EKF 發布 `map -> odom`。
原版 fitness 只供內部閾值判斷和日誌使用，須另外提供品質資訊供 Adapter 閘控。
這個整合尚未實作或驗證。

## 運作模式

- **只有 PGM（目前已實作）：**Local EKF 融合輪式里程計和 IMU；AMCL
  使用 `/scan` 和 PGM 定位，直接發布 `map -> odom`。
- **PGM + PCD（尚未實作）：**AMCL 仍負責主要全域定位；
  FAST_LIO_LOCALIZATION2 以自己的 LiDAR–IMU 里程計和既有 PCD 配準，
  經品質 Adapter 提供額外的 3D 修正，由唯一的全域融合節點發布 `map -> odom`。
- **3D 配準品質不佳：**Adapter 停止發布或提高 covariance，Global EKF 退回以
  AMCL 為主要全域定位來源；停車時的 Global EKF 漂移仍須先查明。
- **地面車限制：**導航主要使用 `x/y/yaw`；`roll/pitch` 以 IMU 為主，`z` 應固定
  或嚴格限制，避免平坦環境中的 3D 配準漂移。

PGM 應由同一份 PCD 投影產生，並保留一致的 `map` 原點、方向與尺度，否則 AMCL
與 FAST_LIO_LOCALIZATION2 的位姿不能直接融合。產生 PGM 與即時虛擬
LaserScan 時，也應使用一致的高度裁切範圍，確保 AMCL 看到的牆面與
2D 地圖相符。

Local EKF 與 Global EKF 可訂閱相同的輪式里程計和 IMU 原始資料，但 Global EKF
不應直接融合 Local EKF 的完整 pose 輸出。這可避免 Global EKF 為了轉換
`odom` frame 的 pose 而依賴自己發布的 `map -> odom`，形成 TF 循環。
FAST-LIO 的局部里程計只供該 3D 定位分支使用，不再當成另一筆獨立的
Global EKF 里程計輸入。
