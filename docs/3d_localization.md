# 2D / 3D 定位與導航架構

AMCL 是主要全域定位來源；當啟動時有提供與 PGM 同座標系的 PCD，
才額外啟動 `hdl_localization`。AMCL 與 HDL 都不發布 TF，而是將位姿交給
Global EKF 融合，確保 `map -> odom` 只有一個發布者。

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
        HDL["hdl_localization<br/>不發布 TF"]
        HDLAdapter["HDL 品質 Adapter<br/>收斂、fitness、時間戳、跳動檢查<br/>設定 covariance"]
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

    %% 有 PCD 時才啟動的 HDL 分支
    PCD -.->|"有提供 PCD 才啟動"| HDL
    Lidar3D --> Deskew
    Deskew -.->|"deskewed PointCloud2"| HDL
    LocalEKF -.->|"initial guess"| HDL
    AMCL -.->|"全域初始化 / 重新定位"| HDL
    HDL -.->|"pose + matching quality"| HDLAdapter
    HDLAdapter -.->|"通過品質檢查的 pose"| GlobalEKF

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
| `map -> odom` | Global `robot_localization` |
| `odom -> base_link` | Local `robot_localization` |
| `base_link -> lidar`、`base_link -> imu` | `robot_state_publisher` 或 static TF |

AMCL 必須設定 `tf_broadcast: false`。`hdl_localization` 也必須關閉 TF
發布功能；實際參數名稱依採用的 ROS 2 port 而定。

## 運作模式

- **只有 PGM：**啟動 AMCL；Local EKF 與 Global EKF 各自使用輪式里程計和 IMU，
  Global EKF 再額外融合 AMCL 的全域位姿。
- **PGM + PCD：**AMCL 仍負責主要全域定位；HDL 通過品質 Adapter 後，提供額外的
  3D 精密修正。
- **HDL 品質不佳：**Adapter 停止發布或提高 covariance，Global EKF 自然退回以
  AMCL 為主要全域定位來源。
- **地面車限制：**導航主要使用 `x/y/yaw`；`roll/pitch` 以 IMU 為主，`z` 應固定
  或嚴格限制，避免平坦環境中的 3D 配準漂移。

PGM 應由同一份 PCD 投影產生，並保留一致的 `map` 原點、方向與尺度，否則 AMCL
與 HDL 的位姿不能直接融合。產生 PGM 與即時虛擬 LaserScan 時，也應使用一致的
高度裁切範圍，確保 AMCL 看到的牆面與 2D 地圖相符。

Local EKF 與 Global EKF 可訂閱相同的輪式里程計和 IMU 原始資料，但 Global EKF
不應直接融合 Local EKF 的完整 pose 輸出。這可避免 Global EKF 為了轉換
`odom` frame 的 pose 而依賴自己發布的 `map -> odom`，形成 TF 循環。
