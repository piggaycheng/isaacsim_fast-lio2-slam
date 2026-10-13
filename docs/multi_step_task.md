# 車端多步任務（py_trees）與步驟放行

`slam_fleet_bridge` 在 Open-RMF 規格之外擴充 `task` 指令：一包 `command` 帶多個步驟，由車端 py_trees 依序執行，**每個步驟之間由 adapter 放行**。MQTT 基本介面見 [online.md](online.md)。

## 設計原則

- 車端若整包自動連跑（導航→雲台→導航），上位系統看不到中間進度，也無法在步驟之間介入。
- 因此車端只負責執行與回報；**下一步何時開始由 adapter 決定**。車端完成一步後停在 `waiting`，收到 `proceed` 才繼續，不會自行往下跑。
- 精細的馬達控制（雲台轉動到位判定）留在車端，上位只看到「步驟完成」。
- `task` 內的 `navigate` 由車端 Nav2 直接規劃，**Open-RMF 不會規劃也無法決定是否放行**（RMF 沒有「詢問某段路能否走」的 API，只協調自己規劃的路徑）。需要 RMF 交通排程的多步任務要改用 RMF compose task，見[上位系統用法](#上位系統用法)。

## 指令

### `task`

```json
{"robot_id":"carter1","cmd_id":42,"action":"task","steps":[
  {"type":"navigate","x":1.0,"y":2.0,"yaw":0.0},
  {"type":"gimbal_move","pan_deg":90,"tilt_deg":10},
  {"type":"navigate","x":5.0,"y":2.0}
]}
```

由 py_trees 建出 `Sequence`，任一步失敗即中止（`aborted`），之後的步驟不執行。步驟內容驗證失敗（未知類型、欄位缺漏等）整包回 `rejected`，不會執行任何步驟。

### `proceed`

```json
{"robot_id":"carter1","cmd_id":42,"action":"proceed"}
```

`cmd_id` 為該 `task` 的 `cmd_id`，放行等待中的任務。只在任務為 `waiting` 時有效，否則忽略並記錄警告（不回 `command_result`，避免 adapter 誤判任務失敗）。

### `gimbal_move`／`gimbal_rotate`（單一雲台動作）

```json
{"robot_id":"carter1","cmd_id":43,"action":"gimbal_move","pan_deg":30,"tilt_deg":-10}
```

與 `navigate` 同樣是獨立的 action type，不需包成 `task`：欄位同下方[步驟類型](#步驟類型)，車端以單步任務執行（不進入 `waiting`），完成回 `command_result`（`completed`／`failed`／`canceled`）。角度不合法回 `failed`。

### `stop` 與新的 `cmd_id`

與單步導航相同：非同步取消，等舊動作（Nav2 或雲台 action）的最終結果後才回報並啟動新任務。`waiting` 期間沒有進行中的 action，取消立即完成並回 `canceled`。

## 步驟類型

| type | 欄位 | 呼叫的 action | 完成條件 |
|---|---|---|---|
| `navigate` | `x`、`y`、`yaw`（`map` 座標，`yaw` 預設 0，rad） | `/NAME/navigate_to_pose`（Nav2） | Nav2 回 `SUCCEEDED` |
| `gimbal_move` | `pan_deg`、`tilt_deg`（度，皆必填） | `/NAME/gimbal/move`（`slam_sensor_control`） | 雲台實測姿態到位（`result.success`） |
| `gimbal_rotate` | `pan_deg`、`tilt_deg`（度，預設 0；`|pan_deg|` ≤ 180） | `/NAME/gimbal/rotate` | 同上，相對目前姿態 |

- 雲台角度單位為度，與 `slam/gimbal/NAME/command` 一致，內部轉為 rad。
- 雲台步驟失敗（任務 `aborted`，`message` 帶雲台回報原因）：tilt 超出限位、逾時未到位（action server 預設 30 s）、沒有雲台狀態，或該車沒有雲台（action server 不存在，訊息含 `no gimbal?`）。
- `rotate`、`take_photo` 已列入協定（`PLANNED_STEP_TYPES`）但尚未實作，含這些步驟的指令整包被拒絕。

## 狀態流程

`task_state`（retained，QoS 1，狀態改變時發布）：

```json
{"robot_id":"carter1","cmd_id":42,"goal_id":"42","status":"waiting","step":0,"steps":3,"message":""}
```

| status | 說明 |
|---|---|
| `running` | 第 `step` 步（0 起算）執行中 |
| `waiting` | 第 `step` 步已完成，等待 `proceed`；下一步尚未開始 |
| `canceling` | 已要求取消，等動作最終結果；不代表已停止，異常附在 `message` |
| `succeeded` / `aborted` / `canceled` / `rejected` | 終態 |

```
task 收到 → running(step0) → waiting(step0) ──proceed──→ running(step1) → waiting(step1) ──proceed──→ running(step2) → succeeded
                                  └─ stop／新 cmd_id → canceling → canceled
```

- 最後一步完成直接 `succeeded`，不需放行；單步 `task` 因此不會進入 `waiting`。
- 終態另發 `command_result`（`succeeded`→`completed`、`aborted`／`rejected`→`failed`、`canceled`→`canceled`）。`waiting` 只出現在 `task_state`，不發 `command_result`。
- `waiting` 時 heartbeat 仍視為任務進行中：`status` 為 `moving`，`current_cmd_id` 保持該任務。

## 上位系統用法

上位系統（`rmf_isaacsim` 的 backend 與 MQTT fleet adapter）提供兩種派送方式，依是否需要 RMF 交通排程選用：

| | RMF compose task（建議） | 直派多步任務 |
|---|---|---|
| backend 入口 | `POST /api/tasks/compose` | `POST /api/tasks/multi-step` |
| 導航由誰規劃 | Open-RMF（`go_to_place`，與其他車協調） | 車端 Nav2，RMF 看不到 |
| 下發到車端 | RMF 逐段下 `navigate`；雲台為 `gimbal_move`／`gimbal_rotate` 指令 | 一包完整 `task` |
| 步驟放行 | RMF 決定每段導航何時出發 | adapter 收到操作員 `proceed` 才放行 |
| 指派車輛 | 可指定，或省略由 RMF 競標 | 必須指定 |
| 適用 | 多車共用路網的正式任務 | 人工操作、測試、不需交通管制的場合 |

### RMF compose task

```json
POST /api/tasks/compose
{"robot_id":"carter1","steps":[
  {"type":"navigate","place":"wp_5"},
  {"type":"gimbal_move","pan_deg":30,"tilt_deg":-10},
  {"type":"navigate","x":-0.2,"y":5.6,"yaw":1.57}
]}
```

backend 把每個步驟轉成一個 compose phase：

- `navigate` → `go_to_place`。須為路網站點：`place` 為站點名稱；只給 `x`/`y` 時對應到 0.5 m 內最近站點，超過即拒絕。`yaw`（rad）帶成到站朝向。
- `gimbal_move`／`gimbal_rotate` → `perform_action`（category 同步驟類型，description 為 `pan_deg`/`tilt_deg`）。fleet adapter 的 action executor 收到後下發同名的 `gimbal_move`／`gimbal_rotate` 指令給車端，`command_result` 為 `completed` 即完成該 phase。fleet 設定的 `actions` 須包含這兩個類型。
- RMF 的 `perform_action` 內不可含 `navigate` 步驟：RMF 會把 action 當作原地執行，車子移動不在交通排程內。adapter 收到這類 action 會拒絕並回報 RMF 失敗。
- 取消走 RMF 的任務取消；RMF 再對車端發 `stop`。

### 直派多步任務

```json
POST /api/tasks/multi-step
{"robot_id":"carter1","steps":[...],"auto_proceed":false}
POST /api/robots/carter1/proceed   {"cmd_id":42}
```

- adapter 下發完整 `task`，依 `task_state` 轉發進度給 backend（`/rmf/task_status`，`step` 為整個任務中的絕對步數）。
- 預設 `auto_proceed=false`：收到 `waiting` 後等操作員 `proceed`，adapter 只在任務為 `waiting` 且 `cmd_id` 相符時才送出。`auto_proceed=true`（可配 `step_delay` 秒）由 adapter 自動放行。
- 車隊暫停或急停時放行先保留，恢復後才補送。
- 車上有 RMF 任務時直派任務會被拒絕（`rejected`），避免搶走 RMF 排定的工作。
- backend 取消會先顯示 `canceling`，等車端回報 `canceled` 才結束。
- 目前 `proceed` 沒有逾時，未放行則車端一直停在 `waiting`（可用 `stop` 結束）。

### adapter 端行為

- `task_state` 與 `command_result` 都帶終態，adapter 以先到者為準，只處理一次。
- RMF 下新的導航／動作／停止時，舊的直派任務先脫離；其後續回報只轉發給 backend，不影響 RMF 新的 execution。
- RMF 擁有的多步任務被急停中斷時，回報 `held`，解除急停後從中斷的步驟續行。
- 任務失敗時記錄錯誤並把車設為 failed；rmf_adapter 沒有「execution 失敗」的 API，execution 保留到在 RMF 取消為止。
- RMF 下發的 `navigate`／`dock` 失敗會自動重試 2 次；`gimbal_move`／`gimbal_rotate` 等動作失敗不重試，直接回報 RMF（限位、無雲台等錯誤重試無效，逾時也已在車端等滿 30 s）。

## 實作

| 檔案 | 內容 |
|---|---|
| `ros2_ws/src/slam_fleet_bridge/scripts/task_bt.py` | 步驟驗證（`STEP_TYPES`）、`ActionStep` 基底與 `NavigateToPose`／`GimbalMove`／`GimbalRotate`、`ReleaseGate`、`TaskRunner` |
| `.../scripts/fleet_bridge_node.py` | 依步驟類型建立 ROS action client，處理 `task`／`proceed`／`stop`，發布 `task_state` 與 `command_result` |
| `.../scripts/rmf_protocol.py` | `COMMAND_ACTIONS` 與指令解析（含 `proceed`、`gimbal_move`／`gimbal_rotate`） |

- `TaskRunner` 以 `Sequence(memory=True)` 組樹：非第一個步驟前插入 `ReleaseGate`，步驟 i 在子節點索引 `2i`；gate 在 `proceed()` 設定 `released` 前回傳 `RUNNING`，`TaskRunner` 以目前子節點是否為 gate 判斷 `waiting`。
- 取消：`ActionStep` 提供 `request_cancel()`／`poll_cancel()`，非同步送出 action 取消並輪詢最終結果；`ReleaseGate` 的取消立即完成。
- `update()` 不可阻塞：action 的 goal 送出與結果取得都走 future 輪詢。
- 新增步驟類型：在 `STEP_TYPES` 加驗證函式，繼承 `ActionStep` 實作 `build_goal()`（必要時覆寫 `succeeded()`、`failure_message()`），登記到 `STEP_BEHAVIOURS`（含 action 名稱），並在 `fleet_bridge_node.py` 的 action 型別對照補上 action 類型。
- 依賴：`package.xml` 需 `slam_sensor_control`（雲台 action 定義）。

## 限制

- 已結束的 `cmd_id` 沒有去重，重送同一個已結束的 `cmd_id` 會重新執行。
- 取消失敗（`canceling` 且 `message` 有錯誤）後，不能重新要求取消。
- `proceed` 無逾時、不重送保證（QoS 1 可能重複，重複的 `proceed` 在非 `waiting` 時被忽略）。

## 測試

- `tests/test_task_bt.py`：步驟解析、放行、`waiting` 取消、取消流程。
- `tests/test_fleet_bridge.py`、`tests/test_rmf_protocol.py`：節點與協定。
- `tests/test_navigation_cancel_ros.py`、`tests/test_task_gimbal_ros.py`：以真實 ROS action server（含 `GimbalActionServer`）驗證導航取消、雲台移動／旋轉、超限與逾時。
