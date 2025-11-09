<img width="437" height="29" alt="image" src="https://github.com/user-attachments/assets/0c5a3e02-e255-45a6-bc91-fbf9b5e8f219" /># unitreego2
something about unitreego2
1. 官方 SDK 地址

GitHub 上的官方 C++ SDK：unitree_sdk2 (Unitree robot sdk version 2) — 地址：https://github.com/unitreerobotics/unitree_sdk2 

其中有专门针对 GO2 的「低层控制（low‑level motor control via DDS topic rt/lowcmd, rt/lowstate）」示例：在 example/go2/go2_low_level.cpp 中。 

注意：高层接口中以前可以用 BodyHeight(float height) 调整机身高度，但在新版 SDK（v2.x）中该接口被删除或不可用。

2. 原理说明（低层控制）

因为高层 “调整机身高度” 接口被删了，你需要直接控制腿上的电机关节，从而达到“半蹲”效果。下面是关键点：

通过 LowCmd_ 消息发布到 rt/lowcmd 话题（DDS）控制电机。 (DeepWiki)

同时订阅 LowState_ 消息从 rt/lowstate 获得当前电机状态。 (GitHub)

在 LowCmd_ 中，每个 motor_cmd[i] 包含以下字段（对 GO2 平台适用）：

mode：电机控制模式（如伺服模式）

q：目标角度（rad）

dq：目标角速度（rad/s）

kp：位置环刚度

kd：速度环阻尼/刚度

tau：前馈力矩（Nm）

<img width="437" height="29" alt="Screenshot 2025-11-09 110541" src="https://github.com/user-attachments/assets/5b9b8300-8ed5-4dbb-9240-e4e3d139b870" />


在 GO2 上，需要 “释放”高层控制模式（比如 sport mode）之后，才能安全地使用低层控制。否则高层可能会干扰。 

通常控制周期比较快，比如 500 Hz (~2 ms) 线程发布控制命令。 

3. 半蹲动作思路

“半蹲”动作其实就是机器人腿部（大腿、小腿、胯）将角度调成一个稍微低一点的高度，比如让机身下降一点但不跪下。你可以选择例如所有腿的“小腿”关节角度减少、或者胯关节向下倾一点，从而机身高度降低。

步骤大致如下：

停止高层运动控制（确保低层控制通道可用）

读取当前各腿关节初始角度（比如 qInit[]）

设定目标角度 qDes[] 为“半蹲”姿势（例如每条腿的大腿‑10°，小腿＋20°等，根据机器人的结构）

设置合适的 KP、KD 控制参数

在控制线程中，渐变（插值）从初始角度过渡到目标角度（避免直接跳变）

发布 LowCmd_ 消息，持续控制直到姿势稳定

如果需要保持 “半蹲”姿势，只要持续发送该姿势命令即可
