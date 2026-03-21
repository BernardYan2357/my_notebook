# MoveIt Task Constructor (MTC) 教程

本教程基于当前工作区中的示例程序：
- src/my_robot_demo/src/moveit_MTC_demo.cpp

目标：
- 使用夹爪抓取一个圆柱体
- 将圆柱体搬运到新位置
- 放置并释放目标

---

## 1. 什么是 MTC

MoveIt Task Constructor（MTC）是 MoveIt 中用于“任务级运动规划”的框架。与只做单次规划不同，MTC 把复杂任务拆成多个阶段（Stage），再组合成完整任务（Task）。

典型流程：
1. 读取当前机器人状态
2. 打开夹爪
3. 移动到抓取位姿
4. 允许夹爪与目标碰撞
5. 闭合夹爪并附着目标
6. 抬升目标
7. 移动到放置位姿
8. 打开夹爪并分离目标
9. 撤离

---

## 2. 本项目中的关键配置

在你的 SRDF 中已经有：
- arm 规划组
- gripper 规划组
- tool_link 末端链接
- gripper_open / gripper_closed 命名状态

这意味着可以直接使用：
- arm 负责空间运动
- gripper 负责开合

---

## 3. 代码结构总览

示例文件主要分为三部分：

1. 创建碰撞物体（圆柱体）
- 函数：makeCylinder
- 使用 PlanningSceneInterface 将圆柱体加入场景

2. 构建 MTC 任务
- 函数：createTask
- 依次添加各个 Stage（MoveTo、MoveRelative、ModifyPlanningScene 等）

3. 主函数执行
- 初始化 ROS2 节点
- 加入圆柱体
- task.plan
- task.execute
- 清理场景物体并退出

---

## 4. 各个 Stage 的作用

### 4.1 CurrentState
读取当前状态，作为任务起点。

### 4.2 MoveTo (open gripper)
将 gripper 移动到 gripper_open。

### 4.3 MoveTo (move to pick)
机械臂移动到抓取预备位姿。

### 4.4 ModifyPlanningScene (allow collision)
临时允许夹爪链接与圆柱体碰撞，避免闭合夹爪时被碰撞检测阻止。

### 4.5 MoveTo (close gripper)
执行夹爪闭合。

### 4.6 ModifyPlanningScene (attach object)
将圆柱体附着到 tool_link，后续规划会把目标当作机器人一部分处理。

### 4.7 MoveRelative (lift object)
沿世界坐标 Z 轴向上抬升目标。

### 4.8 MoveTo (move to place)
将目标搬运到放置位姿。

### 4.9 MoveTo (open gripper for release)
打开夹爪，准备释放目标。

### 4.10 ModifyPlanningScene (detach object)
从 tool_link 分离圆柱体。

### 4.11 MoveRelative (retreat)
末端向上撤离，避免放置后碰撞。

---

## 5. 求解器（Planner/Solver）说明

本示例使用三类求解器：

1. PipelinePlanner
- 用于全局路径规划（如 MoveTo 到某个目标位姿）
- 示例中设置了 RRTConnectkConfigDefault

2. CartesianPath
- 用于笛卡尔直线运动（如抬升、撤离）
- 适合末端沿固定方向移动

3. JointInterpolationPlanner
- 用于关节插值（如夹爪开合）

---

## 6. 运行教程（完整命令）

先编译：

    cd ~/moveit2_ws
    colcon build --packages-select my_robot_demo

加载环境：

    source ~/moveit2_ws/install/setup.bash

终端 1 启动系统：

    ros2 launch my_robot_bringup bringup.launch.xml

终端 2 运行 MTC 演示：

    source ~/moveit2_ws/install/setup.bash
    ros2 run my_robot_demo moveit_MTC_demo

---

## 7. 常见问题与排查

### 7.1 include 报错：找不到 joint_interpolation_planner.h

现象：
- 编译时报错找不到该头文件

原因：
- 当前安装的 MTC 版本头文件名是 joint_interpolation.h

修复：
- 使用以下 include：

    #include <moveit/task_constructor/solvers/joint_interpolation.h>

### 7.2 抓取阶段规划失败

检查项：
1. 抓取位姿是否可达（尤其是 Z 高度和手爪朝向）
2. arm 与 gripper 组名是否和 SRDF 一致
3. 末端链接是否是 tool_link
4. 是否正确执行了 allowCollisions 与 attachObject

### 7.3 执行失败但规划成功

检查项：
1. 控制器是否正常加载
2. 关节限位是否过于严格
3. 轨迹是否过近导致控制器无法稳定执行

---

## 8. 建议你继续做的改进

1. 增加“接近（approach）”阶段
- 在闭合夹爪前，用 MoveRelative 先沿工具坐标系向目标接近

2. 加入 Place 约束
- 在放置阶段加入姿态约束，保证圆柱体竖直放置

3. 参数化目标位置
- 把抓取点、放置点做成 ROS 参数，便于快速调试

4. 增加失败回退策略
- 规划失败时自动尝试备用位姿

---

## 9. 一句话总结

MTC 的核心价值是把复杂任务拆成可解释、可调试的阶段；你当前示例已经具备完整的抓取-搬运-放置闭环，是非常好的进阶基础。