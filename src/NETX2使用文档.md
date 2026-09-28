# Android 端任务与库位 API 说明

本文给 Android 开发使用，主要说明如何通过 HTTP API 管理任务和库位。

## 基本信息

- API 服务地址示例：`http://<host>:6080`
- 例如：`http://10.148.10.253:6080`
- 请求格式：除特别说明外，全部使用 JSON。
- 请求头：`Content-Type: application/json`
- 当前接口没有后端鉴权。

Android 原生请求不受浏览器 CORS 限制；如果使用 WebView，服务端已经返回 CORS headers。

## 通用响应

任务接口通常返回：

```json
{
  "ok": true,
  "message": "ok"
}
```

库位接口通常返回：

```json
{
  "success": true,
  "message": "ok"
}
```

业务上请优先判断 `ok` 或 `success`，失败时展示 `message`。

## 任务状态

任务状态字段为 `status`，常见值：

- `CREATED`：已创建，尚未开始
- `PENDING`：已加入执行队列
- `RUNNING`：执行中
- `SUCCEEDED`：成功
- `FAILED`：失败
- `CANCELED`：已取消
- `PAUSED`：暂停

注意：创建任务后不会自动开始。Android 端需要再调用运行接口。

## 获取任务列表

```http
GET /api/task_manager/tasks
```

响应示例：

```json
{
  "ok": true,
  "message": "cached",
  "tasks": [
    {
      "task_id": "uuid",
      "name": "feeding_task",
      "robot_name": "Huangshan",
      "fleet_name": "nest",
      "status": "CREATED",
      "created_at_unix_ms": 1778995159000,
      "updated_at_unix_ms": 1778995159000
    }
  ]
}
```

## 获取任务详情

```http
GET /api/task_manager/task/<task_id>
```

响应示例：

```json
{
  "ok": true,
  "message": "ok",
  "task": {
    "task_id": "uuid",
    "name": "feeding_task",
    "robot_name": "Huangshan",
    "fleet_name": "nest",
    "status": "RUNNING",
    "created_at_unix_ms": 1778995159000,
    "updated_at_unix_ms": 1778995169000
  },
  "unit_tasks": [
    {
      "unit_id": "uuid",
      "seq": 0,
      "waypoint": "pickup_1",
      "action_name": "feeding_task",
      "status": "RUNNING",
      "attempts": 1,
      "last_error": ""
    }
  ]
}
```

## 创建任务

创建接口当前只返回 `ok/message`，不直接返回 `task_id`。创建成功后请调用 `GET /api/task_manager/tasks` 刷新列表，再按 `robot_name/name/created_at_unix_ms` 找到新任务。

### 创建去充电任务

```http
POST /api/task_manager/create_charger
```

```json
{
  "fleet": "nest",
  "robot": "Huangshan",
  "waypoint": "charger_1",
  "name": "go_to_charger"
}
```

### 创建 dropoff 任务

`waypoint` 必须以 `dropoff` 开头。

```http
POST /api/task_manager/create_dropoff
```

```json
{
  "fleet": "nest",
  "robot": "Huangshan",
  "waypoint": "dropoff_1",
  "name": "dropoff"
}
```

### 创建区域 feeding 任务

机器人会在指定 `shelf_area` 内寻找可投喂的空 cell。

```http
POST /api/task_manager/create_feeding
```

```json
{
  "fleet": "nest",
  "robot": "Huangshan",
  "shelf_area": 1,
  "pickup_waypoint": "pickup_1",
  "dropoff_waypoint": "dropoff_1",
  "name": "feeding_task"
}
```

### 创建货架 feeding 任务

可以选择同一个 area 下的多个 shelf。

```http
POST /api/task_manager/create_shelf_feeding
```

```json
{
  "fleet": "nest",
  "robot": "Huangshan",
  "shelf_area": 1,
  "shelf_indices": [1, 2, 3],
  "pickup_waypoint": "pickup_1",
  "dropoff_waypoint": "dropoff_1",
  "name": "shelf_feeding_task"
}
```

兼容旧字段：

```json
{
  "shelf_index": 1
}
```

## 运行任务

创建任务后调用此接口开始执行。

```http
POST /api/task_manager/run
```

```json
{
  "task_id": "uuid"
}
```

成功响应：

```json
{
  "ok": true,
  "message": "queued"
}
```

## 取消任务

```http
POST /api/task_manager/cancel
```

```json
{
  "task_id": "uuid"
}
```

取消运行中的任务时，后端会尽量向 RMF 和机器人侧转发取消信号。

## 删除任务

```http
POST /api/task_manager/delete
```

```json
{
  "task_id": "uuid"
}
```

删除接口是强制删除，Android 端建议只允许删除终态任务，或者先取消再删除。

## 任务定时计划

### 创建定时计划

```http
POST /api/task_manager/schedule
```

```json
{
  "task_id": "uuid",
  "trigger_type": "CRON",
  "cron_expr": "0 8 * * *",
  "timezone": "Asia/Shanghai"
}
```

### 获取计划列表

```http
GET /api/task_manager/schedules
```

### 更新计划

```http
POST /api/task_manager/schedule_update
```

```json
{
  "schedule_id": "uuid",
  "enabled": true,
  "trigger_type": "CRON",
  "cron_expr": "0 8 * * *",
  "timezone": "Asia/Shanghai"
}
```

### 删除计划

```http
POST /api/task_manager/schedule_delete
```

```json
{
  "schedule_id": "uuid"
}
```

## 获取库位状态

```http
GET /api/storage
```

响应示例：

```json
{
  "layout": [
    {
      "area_index": 1,
      "shelves": [
        {
          "shelf_index": 1,
          "columns": 7,
          "rows": 5,
          "shelf_side": "left"
        }
      ]
    }
  ],
  "cells": {
    "area_1_shelf_1_column_1_row_1": {
      "enabled": true,
      "stock": 0
    }
  },
  "summary": {
    "total": 35,
    "disabled": 0,
    "with_stock": 0
  },
  "timestamp": 1778995169.5
}
```

Cell ID 规则：

```text
area_{area}_shelf_{shelf}_column_{column}_row_{row}
```

所有索引从 `1` 开始。

字段说明：

- `enabled=true`：cell 可用
- `enabled=false`：cell 禁用
- `stock=0`：空
- `stock>0`：有库存
- `shelf_side`：货架在走廊左侧或右侧，值为 `left` / `right`

## 设置库存

### 单个 cell

```http
POST /api/storage/set_stock
```

```json
{
  "cell_id": "area_1_shelf_1_column_1_row_1",
  "stock": 1
}
```

### 批量 cell

```json
{
  "cell_ids": [
    "area_1_shelf_1_column_1_row_1",
    "area_1_shelf_1_column_1_row_2"
  ],
  "stock": 1
}
```

规则：

- `stock` 必须大于等于 `0`
- 批量设置时，禁用 cell 会被跳过

## 启用或禁用库位

```http
POST /api/storage/enable
```

单个 cell：

```json
{
  "cell_id": "area_1_shelf_1_column_1_row_1",
  "enable": false
}
```

批量 cell：

```json
{
  "cell_ids": [
    "area_1_shelf_1_column_1_row_1",
    "area_1_shelf_1_column_1_row_2"
  ],
  "enable": true
}
```

禁用后该 cell 不会被 feeding 任务选择。

## 修改初始库位布局

### 读取当前初始化布局

```http
GET /api/storage/init_yaml
```

响应里会同时返回：

- `content`：原始 YAML 文本
- `layout`：结构化布局，Android 推荐使用这个字段做表单编辑

### 保存初始化布局

推荐用结构化 `layout` 保存，不需要 Android 直接编辑 YAML 文本。

```http
POST /api/storage/init_yaml
```

```json
{
  "layout": [
    {
      "area_index": 1,
      "shelves": [
        {
          "columns": 7,
          "rows": 5,
          "shelf_side": "left"
        },
        {
          "columns": 7,
          "rows": 5,
          "shelf_side": "right"
        }
      ]
    }
  ]
}
```

保存只会更新初始化 YAML 文件，不会立刻重建当前库位。

### 应用初始化布局

```http
POST /api/storage/reinit
```

```json
{}
```

这个接口会按初始化 YAML 重建库位布局，并重置库存状态。Android 端调用前建议弹出确认框。

## 辅助接口

### 获取 fleet 和机器人状态

```http
GET /api/fleets
```

Android 可用这个接口获取当前 fleet 名和 robot 名，用于创建任务时填入 `fleet` 和 `robot`。

### 获取 RMF 任务缓存

```http
GET /api/tasks
```

这个接口返回 RMF 原始任务状态、日志和执行历史。普通任务管理优先使用 `/api/task_manager/*`。

## Android 端建议流程

1. 打开页面或进入任务页时调用 `GET /api/fleets`，拿到 fleet/robot。
2. 每 1 到 3 秒轮询 `GET /api/task_manager/tasks` 更新任务列表。
3. 点进详情时调用 `GET /api/task_manager/task/<task_id>`。
4. 创建任务后刷新任务列表，然后让用户点击“开始”，再调用 `POST /api/task_manager/run`。
5. 库位页面每 1 到 3 秒轮询 `GET /api/storage`。
6. 库位禁用、启用、改库存后，重新拉取 `GET /api/storage` 确认状态。

## curl 示例

```bash
curl -sS http://10.148.10.253:6080/api/task_manager/tasks
```

```bash
curl -sS -X POST \
  -H "Content-Type: application/json" \
  -d '{"fleet":"nest","robot":"Huangshan","waypoint":"dropoff_1","name":"dropoff"}' \
  http://10.148.10.253:6080/api/task_manager/create_dropoff
```

```bash
curl -sS -X POST \
  -H "Content-Type: application/json" \
  -d '{"task_id":"uuid"}' \
  http://10.148.10.253:6080/api/task_manager/run
```

```bash
curl -sS -X POST \
  -H "Content-Type: application/json" \
  -d '{"cell_id":"area_1_shelf_1_column_1_row_1","stock":1}' \
  http://10.148.10.253:6080/api/storage/set_stock
```
