功能包列表：

- my_robot_description：包含机器人模型的URDF文件和用于显示模型的launch文件
- my_robot_moveit_config：由 MoveIt Setup Assistant生成的MoveIt配置包
- my_robot_bringup：包含启动机器人控制所需的launch文件
- my_robot_interfaces：包含定义机器人接口的ROS消息类型文件
- my_robot_commander_cpp：包含一个C++节点，用于订阅机器人控制指令
- my_robot_commander_py：包含一个Python节点，用于订阅机器人控制指令
- my_robot_joints_subscriber: 包含一个C++节点，用于订阅机器人关节位置信息