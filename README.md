# 视觉组第二题
## 此次改动内容
1. 重做了车与地图模型，车由xacro宏定义urdf制作，地图由sdf制作
2. 重写了teleop控制节点
3. 重写几道题的launch文件
4. 创建简易路点节点解决了进入槽内时卡住
## 使用AI的部分
1. 最初自己进行简易的nav2调参，效果不佳，且全向操作不生效，后让AI辅助调参，也没有提升
2. 车子模型中麦克纳姆轮插件设置问题，导致车子横移失效，用agent诊断修改
## 天才想法：
- 由于车子进槽会卡墙，且调参无果，我加了一个简单路点：订阅goal_pos和TF,当起始点或目标点有一个在设定的矩形内时（槽内），就在一秒后发送路点（槽口附近）坐标到goal_pos，30s后默认车子到达路点，就重新发布原始目标点
- 缺点：当车从槽内出去时，会花费大量时间在路点位置等待原始目标点释放，但并非卡住；并且车在槽内移动时会先到路点位置停留
- 解决方式：可在判断部分加分支，把出发点在槽内和目标点在槽内等情况分开处理（理论通过，尚未实行）
## 启动方法
### 第一问&第二问
  - source install/setup.sh
  - -colcon build
  - ros2 launch my_car question1.2.launch.py
### 第三问
  - source install/setup.sh
  - -colcon build
  - ros2 launch my_car question3.launch.py
### 附加题
  - source install/setup.sh
  - -colcon build
  - ros2 launch my_car questionADD.launch.py
  - 发布pose使用2D_Pose_Estimate，发布goal使用**2D_Goal_Pose**,不是**Nav2_Goal**!!!,用错会导致临时路点节点失效


