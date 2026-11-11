# Go2 Web · Go2 Air 可视化控制台

在 macOS 上通过浏览器连接自己的宇树 Go2 Air，查看前方视频、机器狗跟随视角、实时三维体素地图和关节状态，并进行手动遥控、二维建图与目标点导航。

本项目以 `go2_web/` 为主要应用入口，使用 Python WebRTC 服务连接机器人，通过 WebSocket 向浏览器传输状态和媒体数据，前端使用 Three.js 渲染三维场景。开发与排障围绕 Go2 Air 普通版和本机 macOS 环境展开。

项目基于 `legion1581/unitree_webrtc_connect` 开发，三维感知界面借鉴并移植了 `legion1581/unitree_ui` 中的实现。具体来源见文末「来源与致谢」。

## 当前功能

| 功能 | 实现内容 |
| --- | --- |
| 连接与设备发现 | LocalSTA 局域网连接，支持指定 IP 或按序列号自动发现机器人 |
| 实时视频 | 接收机器人前方摄像头画面，在控制台显示 |
| 三维感知 | Go2 模型、实时关节姿态、随航向变化的跟随相机、总览视角切换 |
| 三维地图 | 解码压缩体素数据，使用纹理三角网格渲染实时局部地图 |
| 手动遥控 | 双摇杆、键盘、速度比例调整、普通／AI 模式切换和姿态操作 |
| 避障开关 | 查询和切换机器人避障状态，按状态选择手动驾驶指令通道 |
| 实时建图 | 二维占据栅格、扫描匹配漂移校正、地图清空与位姿显示 |
| 目标点导航 | 在地图上指定位置和朝向，A* 路径规划、路径跟随、受阻重规划与停车检查 |
| 控制状态管理 | 解锁／锁定、停止按钮、页面失焦锁定、控制心跳超时处理 |
| 局域网访问 | 可选 `--lan`，供同一局域网中的 iPad 等设备访问 |

## 环境与连接要求

- **开发环境**为 macOS，当前离线验证使用 Python 3.13.18 和 Chrome。底层包在 `pyproject.toml` 中声明 Python ≥ 3.8，`go2_web` 的其他 Python 版本兼容性仍需验证。
- **机器人**以 Go2 Air 普通版为开发目标，Go2 固件 1.1.15 及以上的本地握手需要设备专属的 32 位十六进制 AES 密钥。
- **网络**采用 LocalSTA，Mac 与机器人连接到同一局域网。Web 控制台当前连接流程使用该模式。
- **设备地址**可直接设置 `UNITREE_ROBOT_IP`，也可设置 `UNITREE_ROBOT_SN` 后自动发现。固件 1.1.15 及以上的定向发现需要序列号，实际结果取决于机器人固件及局域网组播条件。

## 安装

克隆本仓库并进入项目根目录。

```sh
git clone https://github.com/lagrangeluo/unitree_webrtc_connect.git
cd unitree_webrtc_connect
```

首次使用时创建虚拟环境。已有 `venv/` 时直接激活，所有 Python 安装和运行命令均在该环境内执行。

```sh
python3 -m venv venv
source venv/bin/activate
python -m pip install -e .
python -m pip install -r go2_web/requirements.txt
```

macOS 若在安装音频依赖时提示缺少 PortAudio，可先执行 `brew install portaudio`，再重试虚拟环境内的依赖安装。前端依赖和三维资源已包含在仓库中，运行控制台直接使用 Python 服务。

## 配置与启动

以下命令使用 macOS 默认 zsh。在已激活虚拟环境的终端中设置机器人的局域网 IP，并交互输入 AES 密钥。

```sh
export UNITREE_ROBOT_IP='你的机器人局域网IP'
read -s 'UNITREE_AES_KEY?请输入设备 AES 密钥: '
echo
export UNITREE_AES_KEY
python -m go2_web
```

浏览器打开 **http://127.0.0.1:8765**，点击「连接机器人」后开始接收数据。重新打开终端后，需要再次激活虚拟环境并设置环境变量。

如需自动发现地址，用序列号配置替代指定 IP，然后启动服务。

```sh
unset UNITREE_ROBOT_IP
read -r 'UNITREE_ROBOT_SN?请输入机器人序列号: '
export UNITREE_ROBOT_SN
python -m go2_web
```

AES 密钥可通过底层库提供的工具，从绑定该机器人的宇树账号查询。以下示例适用于中国区 Go2 账号，密码会交互输入，其他地区使用 `--region global`。

```sh
python -m unitree_webrtc_connect._cli --email '你的账号邮箱' --region cn --device-type Go2
```

账号密码、令牌、AES 密钥和真实序列号只用于本地配置。提交代码和分享日志前请移除这些信息。

需要同一局域网中的其他设备访问时，使用下面的启动命令，并打开终端打印的局域网地址。

```sh
python -m go2_web --lan
```

`--lan` 模式没有用户鉴权，同一网络中的设备可接管控制，请仅在可信局域网内使用。新打开的控制页面会接管连接，旧页面会断开。

## 使用

1. 确认机器人已开机、Mac 与机器人处于同一局域网，并关闭其他占用 WebRTC 连接的客户端。
2. 点击「连接机器人」，查看视频、位姿、关节和地图是否持续更新。
3. 三维感知默认跟随机器狗，点击「切换总览」或双击画布进入总览，可旋转、平移和缩放，再次切换返回跟随。
4. 需要移动时，在空旷场地解锁控制。左摇杆控制前后与侧移，右摇杆控制转向，键盘 ↑／↓ 控制前后、←／→ 控制转向、A／D 控制侧移。速度滑条默认 50%。
5. 实时建图面板可查看二维地图与当前位姿。解锁后点击「导航目标」，在已观测的可通行区域按下设置位置、拖动设置朝向，松开发送目标。
6. 使用停止按钮或空格停止并锁定。页面失焦、心跳超时和导航数据过期也会触发控制状态处理。

软件停止依赖连接和机器人响应，实机操作时应保留可用遥控器。目标导航处于实验阶段，低矮障碍、玻璃、镜面、动态障碍与里程计漂移都可能影响通行判断，需低速实测。

详细的指令行为、建图参数、导航停止条件与排障说明见 [Web 控制台使用说明](go2_web/README.md)。

## 代码结构

```text
go2_web/
  __main__.py             Python 服务、连接、状态与控制接口
  slam.py                 二维建图、漂移校正、路径规划与导航
  static/                 Web 页面、Three.js 场景、模型与 WASM
  checks/                 离线回归检查
unitree_webrtc_connect/   基于上游的 WebRTC 通信库
examples/                上游保留的 Go2、G1、R1 示例
```

底层库保留上游 G1、R1 等接口与示例，当前 Web 应用的开发目标和验证范围围绕 Go2 Air。

## 验证范围

在项目根目录使用现有虚拟环境执行语法检查和离线回归检查。

```sh
venv/bin/python -m compileall -q go2_web unitree_webrtc_connect
venv/bin/python -m unittest discover -s go2_web/checks -v
```

最近一次检查在 macOS 27.0.1、Python 3.13.18 下完成，42 项离线回归检查通过，覆盖设备发现、点云链路、驾驶限速、避障通道、建图与导航状态处理。三维界面曾使用 Chrome 和离线压缩体素数据验证模型加载、WASM 解码、网格渲染及视角切换。

上述离线检查未连接机器人，机器人固件未核验。实际连接模式配置为 LocalSTA，视频、实机地图对齐、运动与导航效果需要在自己的 Go2 Air 上分别验收。

## 来源与致谢

本仓库在开源项目基础上开发，主要借鉴与移植关系如下。

- **`legion1581/unitree_webrtc_connect`** 是本仓库克隆与后续开发的基础，提供 Python WebRTC 连接、数据通道、消息订阅与 RPC、媒体流、设备发现、鉴权及 LiDAR 解码。本项目在此基础上增加了 `go2_web` 控制台、地图与导航功能，并调整了点云消息处理。[原始通信库仓库](https://github.com/legion1581/unitree_webrtc_connect)
- **`legion1581/unitree_ui`** 提供了三维感知界面的参考实现。本项目借鉴并移植其中的 `scene.ts`、`robot-model.ts`、`voxel-map.ts` 和 `voxel-worker.ts`，沿用 Go2 模型、体素纹理、HDR 环境及 WASM 资源，并适配到 Python 服务与浏览器 WebSocket 数据链路。[原始可视化界面仓库](https://github.com/legion1581/unitree_ui)
- **Three.js** 用于浏览器三维渲染，仓库内包含其运行时及相关加载器、控制器。[Three.js 仓库](https://github.com/mrdoob/three.js)

项目采用 MIT 许可证，保留上游版权声明。具体条款见 [项目许可证](LICENSE)、[Unitree UI 许可证](go2_web/static/three/UNITREE_UI_LICENSE.txt) 和 [Three.js 许可证](go2_web/static/three/LICENSE.txt)。
