一、系统组成：
lift_robot(courier robot), AMR, UR15

二、demo task必需的启动文件：
D:\MyWorks\MSRA\robot_dc\scripts\ThirdParty\seer_control\app_dc_demo_2025.py
D:\MyWorks\MSRA\robot_dc\colcon_ws\src\ur15_web\launch\ur15_bringup.py
D:\MyWorks\MSRA\robot_dc\colcon_ws\src\robot_bringup\launch\lift_robot_bringup.py
D:\MyWorks\MSRA\robot_dc\colcon_ws\src\system_monitor\launch\system_monitor_launch.py

三、system_monitor_launch相应的网页说明：
1. 左侧Navigation Panel对应的是三个机器人以及ur15_bringup.py启动的各个service（从robot_config加载）
2. 左侧Position Control Panel里的task position和home position是可以互相按照J6->J1顺序运动到特定点位的，home position是直立状态，用于防止运输过程中的碰撞，task position主要是用于执行操作任务之前进行一个规避奇异点的修正位置。具体的可以参考D:\MyWorks\MSRA\robot_dc\scripts\ur_move_to_target.py单步运行。
3. 左侧的Demo Operation Panel中的step对应D:\MyWorks\MSRA\robot_dc\scripts\demo_task_manager.py中的execute_complete_task_sequence中的步骤。其具体的step通过引用这个py文件中的基类，然后基本都是调用相应的execute执行函数来实现的。（这个panel中可以execute selected step和execute all）。
4. 右侧UR15 robot monitor和force data都是通过解析30004这个RTDE通信端口的字段进行实时更新的，具体可以参考D:\MyWorks\MSRA\robot_dc\scripts\ur_rtde_monitor.py的实现。
5. 右侧的lift_robot的monitor主要是通过泽培的webapi的get_status返回值进行更新的。

四、demo有关脚本关系说明：
1. 这里demo_task_manager.py主要调用了D:\MyWorks\MSRA\robot_dc\scripts\demo_amr_functions.py的类来控制AMR，调用D:\MyWorks\MSRA\robot_dc\scripts\courier_robot_webapi.py的类来控制CourierRobot，调用D:\MyWorks\MSRA\robot_dc\scripts\ur_operate_tools.py中的类来管理工具的调用（函数名字感觉写的可以看懂）。调用scripts/ur_wobj_*.py --server-index xx来实现相应的operation（顺序是unlock_knob->open_handle->close_left->close_right->extract_server->put_frame->insert_server->unlock_knob_insert->close_handles）。每一个子脚本都可以在robot_status_redis上有正确的rack positioning结果后独立运行。并且可以通过输入参数server-index进行换层，现在实验室最上层那个坏的服务器的index=16。
2. D:\MyWorks\MSRA\robot_dc\scripts\urscripts_function_test.py这个脚本中是远程模式下URScripts一些基本函数的测试脚本。
3. D:\MyWorks\MSRA\robot_dc\colcon_ws\src\ur15_robot_arm\ur15_robot_arm\ur15.py这个是我自己写的封装库，包括FTC的、freedrive以及movej、move_tcp等基础函数。
4. D:\MyWorks\MSRA\robot_dc\scripts\ur_dashboard_control.py这个可以通过29999这个port替代示教器上的基础控制操作。
5. 我今天给您发的那个workspace.zip里有我的dataset文件夹和temp文件夹（里面有workflow_file有一个简化的rack location demo，以及scripts中某些脚本会生成的调试文件）

五、一些我觉得可以继续改进的地方：
1. ur15 web左侧的运动控制panel，如果tcp和joint angles的数据卡了（不实时更新了），输入或者改变当前值时，ur15仍然会动到目标位置。这里需要小心数据卡死时大范围移动之后修改那个值，机械臂的大范围移动。
2. robot_config.yaml里有一些重复的内容可以删去，可能我和泽培协作的时候没有检查有没有前面已经有了。
3. ur_wobj_*.py中sequence函数的注释小部分可能不是正确的，因为后面调试的时候可能没有及时改。