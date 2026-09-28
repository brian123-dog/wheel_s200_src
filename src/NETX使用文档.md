# NEST X 使用文档
## 概述
NEST 系列控制器提供网页版部署软件和 API 开发接口。用户通过部署软件建图、设置路径点、路径和任务。部署完成后可以通过 API 获取部署数据，并对机器人进行控制。

## 1. API
### 1.1 机器人控制
#### 1.1.1 控制盒心跳
- 协议：TCP/IP
- 地址：<ip>:9999
- 说明：上电几秒后收到心跳，断电即停止
Python代码示例：
import socket
def receive_heartbeat(server_ip, port):
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.connect((server_ip, port))
        print("Connected to server. Listening for heartbeats...")
        try:
            while True:
                data = sock.recv(1024)
                if not data:
                    break
                print("Heartbeat received")
        except Exception as e:
            print(f"Error: {e}")
if __name__ == "__main__":
    receive_heartbeat('192.168.12.1', 9999)

#### 1.1.2 Endpoints
##### 获取机器人数据
- 协议：websocket
- 地址：ws://<api base url>:1234/robot_data
- 频率：10HZ（服务器→客户端）
数据返回样本：
{
    "fsm": "idle",
    "pose": {
        "orientation": {
            "w": 1.0,
            "x": 0.0,
            "y": 0.0,
            "z": -0.0
        },
        "position": {
            "x": 0.01,
            "y": 0.2,
            "z": 0.0
        },
        "pyr": {
            "pitch": 0.0,
            "roll": 0.0,
            "yaw": 0.0
        }
    },
    "speed": {
        "linear": 0.2,
        "angular": 0.0
    },
    "nav_collision": false,
    "localization_error": false,
    "localization_quality": 100,
    "actuator_command_a": 0,
    "current_task_uid": null,
    "current_wp_uid": "-1",
    "system_on": true,
    "imu_on": true,
    "battery_on": true,
    "plc_on": true,
    "wheel_odom_on": true,
    "lidar_on": true,
    "cm": 0
}
FSM状态：
- idle：空闲
- moving：运动中
- succeeded：成功
- failed：失败
- canceled：取消

##### Python WebSocket客户端
import asyncio
import websockets
async def connect_to_robot_data():
    uri = "ws://192.168.12.1:1234/robot_data"
    try:
        async with websockets.connect(uri) as websocket:
            print("Connected to WebSocket server")
            while True:
                message = await websocket.recv()
                print(f"Received message: {message}")
    except websockets.exceptions.ConnectionClosed as e:
        print(f"Connection closed: {e}")
    except Exception as e:
        print(f"An error occurred: {e}")
if __name__ == "__main__":
    asyncio.run(connect_to_robot_data())

##### C# WebSocket客户端
using System;
using System.Net.WebSockets;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
public class WebSocketClient
{
    private const int ReceiveBufferSize = 1024;
    private static ClientWebSocket _webSocket;
    public static async Task Connect(string url)
    {
        _webSocket = new ClientWebSocket();
        await _webSocket.ConnectAsync(new Uri(url), CancellationToken.None);
        Console.WriteLine("Connected to WebSocket server.");
        await Task.WhenAll(Receive(), Send());
    }
    private static async Task Receive()
    {
        var buffer = new byte[ReceiveBufferSize];
        while (_webSocket.State == WebSocketState.Open)
        {
            var result = await _webSocket.ReceiveAsync(new ArraySegment<byte>(buffer), CancellationToken.None);
            if (result.MessageType == WebSocketMessageType.Text)
            {
                var msg = Encoding.UTF8.GetString(buffer, 0, result.Count);
                Console.WriteLine("Received: " + msg);
            }
            else if (result.MessageType == WebSocketMessageType.Close)
            {
                await _webSocket.CloseAsync(WebSocketCloseStatus.NormalClosure, "", CancellationToken.None);
                Console.WriteLine("Closed");
            }
        }
    }
    private static async Task Send()
    {
        while (_webSocket.State == WebSocketState.Open)
        {
            var msg = Console.ReadLine();
            var buf = Encoding.UTF8.GetBytes(msg);
            await _webSocket.SendAsync(new ArraySegment<byte>(buf), WebSocketMessageType.Text, true, CancellationToken.None);
        }
    }
    public static async Task Main()
    {
        var url = "ws://192.168.12.1:1234/robot_data";
        await Connect(url);
    }
}

##### 获取路径
- 协议：websocket
- 地址：ws://<api base url>:1234/path_plan
- 频率：1HZ

##### 速度控制
- 协议：websocket
- 地址：ws://<api base url>:1234/velocity_control
- 频率：10-20HZ（客户端→服务器）
发送数据：
{
    "linear": 0.2,
    "linear_y": 0.0,
    "angular": 0.1
}

#### 设置任务ID
- 协议：HTTP POST
- 地址：http://<api base url>:1234/set_task_id
请求体：
{
    "task_uid": "xxx"
}

#### 机器人直线行走与原地旋转
- 协议：HTTP POST
- 地址：http://<api base url>:1234/motion_control
- 前提：仅idle状态可用
请求体：
{
    "direction": 1,
    "distance": 1.0,
    "speed": 0.2
}
- direction：1=前、2=后、3=左、4=右；distance：米(直行)/rad(旋转)
响应：
{
    "code": 0,
    "message": ""
}

#### 机器人前往位置点
- 协议：HTTP POST
- 地址：http://<api base url>:1234/go_to
- 前提：仅idle状态可用
请求体：
{
    "pose": {
        "position": {
            "x": 2.0,
            "y": 1.0
        },
        "orientation": {
            "x": 0.0,
            "y": 0.0,
            "z": 0.0,
            "w": 1.0
        },
        "pyr": {
            "pitch": 0.0,
            "roll": 0.0,
            "yaw": 0.0
        }
    },
    "use_pyr": true,
    "precision_xy": 0.05,
    "precision_yaw": 0.05,
    "is_reverse": false,
    "nav_type": "auto"
}
}
响应：
{
    "code": 0,
    "message": ""
}

#### 机器人前往多位置点（执行任务）
- 协议：HTTP POST
- 地址：http://<api base url>:1234/execute_task
- 前提：仅idle状态可用
请求体：
{
    "task_uid": "task_001",
    "is_repeat": false,
    "wps": [
        {
            "pose": {
                "position": {
                    "x": 1.0,
                    "y": 2.0
                }
            },
            "is_dest": true,
            "nav_type": "auto",
            "precise_xy": 0.05,
            "precise_rad": 0.05,
            "is_reverse": false
        }
    ]
}
响应：
{
    "code": 0
}

#### 取消当前任务
- 协议：HTTP GET
- 地址：http://<ip>:1234/cancel_task
- 前提：仅moving状态可用
响应：
{
    "code": 0
}

#### 确认机器人状态
- 协议：HTTP GET
- 地址：http://<ip>:5000/deploy/confirmStatus
- 前提：succeeded/failed/canceled后调用
响应：
{
    "code": 0
}

#### 设置举升电机高度
- 协议：HTTP POST
- 地址：http://<ip>:1234/set_lift_height
请求体：
{
    "height": 500
}
响应：
{
    "code": 0
}

#### 机器人重定位
- 协议：HTTP POST
- 地址：http://<ip>:1234/set_pose
请求体：
{
    "use_pyr": true,
    "position": {
        "x": 0.0,
        "y": 0.0
    },
    "pyr": {
        "yaw": 0.0
    },
    "use_absolute": true
}
响应：
{
    "code": 0
}

### 1.2 程序
#### 1.2.1 API Base Url
- 192.168.8.110

#### 1.2.2 安全验证
- Tbc

#### 1.2.3 Endpoints
##### 启动建图程序
- 协议：HTTP GET
- 地址：http://<api base url>:5000/mapping/<res>/<model>
- res：2cm/5cm；model：diff/ackermann
响应：
{
    "code": 0
}

##### 启动导航程序
- 协议：HTTP GET
- 地址：http://<api base url>:5000/navigation/<map_id>/<model>
响应：
{
    "code": 0
}

##### 关闭程序
- 协议：HTTP GET
- 地址：http://<api base url>:5000/stopAndRemoveProgram

##### 程序是否启动
- 协议：HTTP GET
- 地址：http://<api base url>:5000/isRunning
响应：
{
    "code": 0,
    "message": "navigation"
}

##### 获取当前运行地图
- 协议：HTTP GET
- 地址：http://<api base url>:5000/deploy/getCurrentMap
响应：
{
    "code": 0,
    "message": {
        "name": "test",
        "id": 1
    }
}

### 1.3 地图与部署
#### 1.3.1 API Base Url
- 192.168.8.110:5000

#### 1.3.2 安全验证
- Tbc

#### 1.3.3 Endpoints
##### 获取地图列表
- 协议：HTTP GET
- 地址：http://<api base url>/deploy/getMaps
响应：
{
    "code": 0,
    "data": [
        {
            "id": 1,
            "name": "test1"
        },
        {
            "id": 2,
            "name": "test2"
        }
    ]
}

##### 执行任务
- 协议：HTTP POST
- 地址：http://<api base url>/deploy/executeTask
请求：
{
    "uid": "task_001",
    "repeat": false,
    "do_reverse": false
}

##### 获取行为列表
- 协议：HTTP GET
- 地址：http://<api base url>:5000/deploy/getAllActions
响应：
{
    "code": 0,
    "data": [
        {
            "id": 1,
            "name": "action1"
        }
    ]
}

##### 执行行为
- 协议：HTTP GET
- 地址：http://<api base url>:5000/deploy/executeAction/<action_id>/<task_id>

##### 导航时获取地图数据与部署
- 协议：HTTP GET
- 地址：http://<api base url>/deploy/getMapDataWithDetailRos/<map_id>

##### 导航时更换地图
- 协议：HTTP POST
- 地址：http://<ip>:5000/deploy/changeMap
请求：
{
    "map_id": 1
}

##### 前往路径点
- 协议：HTTP POST
- 地址：http://<ip>:5000/deploy/goToWP
请求：
{
    "deploy_uid": "d1",
    "wp_uid": "w1",
    "precision_xy": 0.05,
    "precision_yaw": 0.05,
    "is_reverse": false,
    "nav_type": 1
}

##### 基于路径点重定位
- 协议：HTTP POST
- 地址：http://<api base url>/deploy/relocateWithWP
请求：
{
    "deploy_uid": "d1",
    "wp_uid": "w1"
}

##### 保存地图
- 协议：HTTP GET
- 地址：http://<api base url>:5000/deploy/saveMap/<map_name>

##### 删除地图
- 协议：HTTP GET
- 地址：http://<api base url>:5000/deploy/deleteMap/<map_id>

### 1.4 启动后参数修改
#### 1.4.1 避障区域
- 协议：HTTP POST
- 地址：http://<api base url>:5000/parameter/setCollisionMonitor/
请求：
{
    "name": "Stop",
    "linear_limit": 0.0,
    "angular_limit": 0.0,
    "points": [0,0,1,0,1,1,0,1]
}

#### 1.4.2 开关避障区域
- 协议：HTTP GET
- 地址：http://<api base url>:5000/parameter/enableCollisionMonitor/<enable>
- enable：1=开、0=关

#### 1.4.3 设置最大速度
- 协议：HTTP POST
- 地址：http://<api base url>:5000/parameter/setControllerMaxSpeed/
请求：
{
    "max_speed": 1.0,
    "max_reverse_speed": 0.5
}

### 1.5 ROS通信
（略）

### 1.6 语音
- Base URL：http://<ip>:5000/sound/
- Header：x-api-key:1234567890
#### 1.6.1 获取声音文件列表
- GET /files
#### 1.6.2 上传
- POST /upload（multipart/form-data）
#### 1.6.3 删除
- DELETE /delete/<filename>
#### 1.6.4 播放
- GET /play_local/<filename>
#### 1.6.5 停止播放
- GET /stop_playback

### 1.7 系统
#### 1.7.1 获取当前IP
- GET http://<api base url>:5000/system/getNetworkIPs
#### 1.7.2 获取历史WiFi
- GET http://<api base url>:5000/system/listHistoryWifi
#### 1.7.3 删除历史WiFi
- POST http://<api base url>:5000/system/deleteHistoryWifi
#### 1.7.4 连接WiFi
- POST http://<api base url>:5000/system/connectWifiJson
请求：
{
    "ssid": "wifi",
    "password": "123456"
}
#### 1.7.5 设置热点
- POST http://<api base url>:5000/system/setAPJson
#### 1.7.6 获取网卡列表
- GET http://<api base url>:5000/system/listNetworkInterfaces
#### 1.7.7 修改网络配置
- POST http://<api base url>:5000/system/configureNetworkInterface
#### 1.7.8 关机
- GET http://<api base url>:5000/system/poweroff

### 1.8 全覆盖路径任务
- Base URL：http://localhost:1235
#### 1.8.1 启动导航任务
- POST /navigate_coverage
请求体：
{
    "field": [[0,0],[2,0],[2,2],[0,2]],
    "swath_angle": 0.0,
    "repeat_times": 1
}
#### 1.8.2 获取任务状态
- GET /status
#### 1.8.3 取消任务
- GET /cancel

### 1.9 定时任务
- Base URL：http://<ip>:5000/schedule/
#### 1.9.1 获取所有定时任务
- GET /schedules
#### 1.9.2 创建定时任务
- POST /schedules
请求体：
{
    "type": "weekly",
    "start_time": "10:00:00",
    "days": [0,2,4],
    "task_name": "full_coverage"
}
#### 1.9.3 获取/更新/删除定时任务
- GET /schedules/<id>；PUT /schedules/<id>；DELETE /schedules/<id>

## 2 MQTT
### 2.1 连接Broker
- host：mqtt.example.com；port：1885
### 2.2 电池数据订阅
- 激活：publish battery/sub；订阅： subscribe battery/data
数据示例：
{
    "voltage": 27.2,
    "current": -0.4,
    "charge": 26.22,
    "percentage": 0.87
}

## 3 UI
### 3.1 部署UI
- 浏览器访问：192.168.8.110:5000；默认账号密码：（略）
#### 3.1.1 建图
- 连接网线→进入建图→控制机器人移动→保存地图
#### 3.1.2 路径点
- 选择地图→添加路径点→编辑任务→启动任务
#### 3.1.3 任务执行
- 状态：idle→moving→succeeded/failed/canceled→confirmStatus→idle

## 4 行为
- io控制、回桩、上货等（可配置xml）

## 5 地图编辑
- 软件：GIMP；黑白图：黑=禁行、白=可通行；下载→编辑→上传

## 6 附录
- 麦克纳姆轮、坐标系、图示（略）