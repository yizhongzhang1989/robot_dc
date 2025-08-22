#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
工具状态切换系统
管理四种工具（gripper, frame, stickP, stickR）的状态切换
状态表示：每个位代表一个工具的状态，1表示在原位，0表示被取出
格式：[gripper, frame, stickP, stickR]
"""

class ToolStateManager:
    def __init__(self):
        # 初始状态：所有工具都在原位
        self.current_state = [1, 1, 1, 1]  # [gripper, frame, stickP, stickR]
        
        # 定义按钮操作
        self.button_operations = {
            'A': [0],     # gripper位置取反
            'B': [1],     # frame位置取反
            'C': [0, 2],  # gripper置0，stickP取反
            'D': [0, 3]   # gripper置0，stickR取反
        }
        
        # 定义有效状态（根据物理约束分析）
        # stickP和stickR只有在gripper被取出时才可以夹取
        self.valid_states = {
            (1, 1, 1, 1),  # 1111 - 默认状态，所有工具在原位
            (0, 1, 1, 1),  # 0111 - 只取gripper（正在使用gripper）
            (1, 0, 1, 1),  # 1011 - 只取frame（正在使用frame）
            (0, 1, 0, 1),  # 0101 - 取gripper后夹取stickP（准备使用stickP）
            (0, 1, 1, 0),  # 0110 - 取gripper后夹取stickR（准备使用stickR）
            (0, 0, 1, 1),  # 0011 - 先取frame，再取gripper
            (0, 0, 0, 1),  # 0001 - 先取frame，再取gripper，然后夹取stickP
            (0, 0, 1, 0),  # 0010 - 先取frame，再取gripper，然后夹取stickR
        }
    
    def state_to_string(self, state):
        """将状态转换为字符串表示"""
        return ''.join(map(str, state))
    
    def is_valid_state(self, state):
        """检查状态是否有效"""
        return tuple(state) in self.valid_states
    
    def get_current_state_string(self):
        """获取当前状态的字符串表示"""
        return self.state_to_string(self.current_state)
    
    def apply_action(self, state, action_name):
        """在给定状态上应用动作"""
        new_state = state.copy()
        
        if action_name == 'zero2gripper':
            new_state[0] = 0  # gripper被取出
        elif action_name == 'gripper2zero':
            new_state[0] = 1  # gripper放回
        elif action_name == 'zero2frame':
            new_state[1] = 0  # frame被取出
        elif action_name == 'frame2zero':
            new_state[1] = 1  # frame放回
        elif action_name == 'zero2stickP':
            new_state[2] = 0  # stickP被取出
        elif action_name == 'stickP2zero':
            new_state[2] = 1  # stickP放回
        elif action_name == 'zero2stickR':
            new_state[3] = 0  # stickR被取出
        elif action_name == 'stickR2zero':
            new_state[3] = 1  # stickR放回
        
        return new_state
    
    def calculate_target_state(self, button):
        """根据按钮操作计算目标状态"""
        if button not in self.button_operations:
            return None
        
        target_state = self.current_state.copy()
        
        if button == 'A':  # gripper取反
            target_state[0] = 1 - target_state[0]
        elif button == 'B':  # frame取反
            target_state[1] = 1 - target_state[1]
        elif button == 'C':  # gripper置0，stickP取反
            target_state[0] = 0  # gripper强制取出
            target_state[2] = 1 - target_state[2]  # stickP取反
        elif button == 'D':  # gripper置0，stickR取反
            target_state[0] = 0  # gripper强制取出
            target_state[3] = 1 - target_state[3]  # stickR取反
        
        # 检查目标状态是否有效
        if tuple(target_state) in self.valid_states:
            return target_state
        
        # 如果状态无效，需要应用约束逻辑
        return self.apply_constraints_for_toggle(self.current_state, button)
    
    def apply_constraints_for_toggle(self, current_state, button):
        """应用约束逻辑来处理按钮操作"""
        target_state = current_state.copy()
        
        if button == 'A':  # gripper取反
            target_state[0] = 1 - target_state[0]
            # 如果gripper要被放回，那么stickP和stickR也必须放回
            if target_state[0] == 1:  # gripper被放回
                target_state[2] = 1  # stickP必须放回
                target_state[3] = 1  # stickR必须放回
            
        elif button == 'B':  # frame取反
            # 关键约束：gripper正在使用时，不能操作frame
            if current_state[0] == 0:  # gripper取出状态
                # 先计算理想的frame取反结果
                ideal_frame_state = 1 - current_state[1]
                
                # 必须先放回所有工具到1111状态，然后重新按需取出
                target_state = [1, 1, 1, 1]  # 先全部放回
                
                # 然后设置最终需要的状态
                target_state[1] = ideal_frame_state  # frame按需求设置
                
                # 如果之前有stickP或stickR被取出，需要重新取出gripper和对应的stick
                if current_state[2] == 0:  # 之前stickP被取出
                    target_state[0] = 0  # 重新取出gripper
                    target_state[2] = 0  # 重新取出stickP
                elif current_state[3] == 0:  # 之前stickR被取出
                    target_state[0] = 0  # 重新取出gripper  
                    target_state[3] = 0  # 重新取出stickR
            else:
                # gripper在位时可以正常操作frame
                target_state[1] = 1 - target_state[1]
                
        elif button == 'C':  # gripper置0，stickP取反
            target_state[0] = 0  # gripper强制取出
            
            # 如果stickR已经取出，需要先放回stickR（互斥约束）
            if current_state[3] == 0:
                target_state[3] = 1  # 放回stickR
            
            target_state[2] = 1 - current_state[2]  # stickP取反
                
        elif button == 'D':  # gripper置0，stickR取反
            target_state[0] = 0  # gripper强制取出
            
            # 如果stickP已经取出，需要先放回stickP（互斥约束）
            if current_state[2] == 0:
                target_state[2] = 1  # 放回stickP
            
            target_state[3] = 1 - current_state[3]  # stickR取反
        
        return target_state
    
    def is_action_allowed(self, current_state, action_name):
        """检查在当前状态下是否允许执行某个动作"""
        # 关键约束：gripper正在使用时，不能操作frame（取出或放回）
        if (action_name == 'zero2frame' or action_name == 'frame2zero') and current_state[0] == 0:  # gripper已取出时不能操作frame
            return False
        
        # stickP和stickR只有在gripper被取出时才能夹取
        if action_name == 'zero2stickP' and current_state[0] == 1:  # gripper在位时不能取stickP
            return False
        
        if action_name == 'zero2stickR' and current_state[0] == 1:  # gripper在位时不能取stickR
            return False
        
        # 不能同时取stickP和stickR
        if action_name == 'zero2stickP' and current_state[3] == 0:  # stickR已取出时不能取stickP
            return False
        
        if action_name == 'zero2stickR' and current_state[2] == 0:  # stickP已取出时不能取stickR
            return False
        
        return True

    def find_shortest_path(self, target_button):
        """使用BFS找到从当前状态到目标状态的最短路径"""
        if target_button not in self.button_operations:
            return None, f"无效的目标按钮: {target_button}"
        
        target_state = tuple(self.calculate_target_state(target_button))
        current_state = tuple(self.current_state)
        
        # 如果已经在目标状态
        if current_state == target_state:
            return [], f"已经在目标状态 {self.state_to_string(target_state)}"
        
        # BFS搜索
        from collections import deque
        
        queue = deque([(current_state, [])])  # (状态, 路径)
        visited = {current_state}
        
        # 定义所有可能的动作
        actions = [
            ('zero2gripper', lambda s: (0, s[1], s[2], s[3]) if s[0] == 1 else None),
            ('gripper2zero', lambda s: (1, s[1], s[2], s[3]) if s[0] == 0 else None),
            ('zero2frame', lambda s: (s[0], 0, s[2], s[3]) if s[1] == 1 else None),
            ('frame2zero', lambda s: (s[0], 1, s[2], s[3]) if s[1] == 0 else None),
            ('zero2stickP', lambda s: (s[0], s[1], 0, s[3]) if s[2] == 1 else None),
            ('stickP2zero', lambda s: (s[0], s[1], 1, s[3]) if s[2] == 0 else None),
            ('zero2stickR', lambda s: (s[0], s[1], s[2], 0) if s[3] == 1 else None),
            ('stickR2zero', lambda s: (s[0], s[1], s[2], 1) if s[3] == 0 else None),
        ]
        
        while queue:
            current, path = queue.popleft()
            
            # 尝试所有可能的动作
            for action_name, action_func in actions:
                new_state = action_func(current)
                
                # 检查动作是否有效且新状态是否有效
                if new_state is not None and new_state in self.valid_states and new_state not in visited:
                    # 额外约束检查：确保动作符合业务逻辑
                    if self.is_action_allowed(current, action_name):
                        new_path = path + [action_name]
                        
                        # 如果到达目标状态
                        if new_state == target_state:
                            return new_path, f"成功切换到状态 {self.state_to_string(target_state)}"
                        
                        queue.append((new_state, new_path))
                        visited.add(new_state)
        
        # 如果没有找到路径
        return None, "无法从当前状态切换到目标状态"
    
    def execute_button_action(self, button):
        """执行按钮动作"""
        path, message = self.find_shortest_path(button)
        
        if path is None:
            print(f"错误: {message}")
            return False
        
        if not path:
            print(f"信息: {message}")
            return True
        
        target_state_list = self.calculate_target_state(button)
        print(f"当前状态: {self.get_current_state_string()}")
        print(f"目标状态: {self.state_to_string(target_state_list)}")
        print(f"执行动作: {' -> '.join(path)}")
        
        # 执行路径中的每个动作
        for action in path:
            self.current_state = self.apply_action(self.current_state, action)
        
        return True
    
    def reset_to_default(self):
        """重置到默认状态"""
        current_state_str = self.get_current_state_string()
        target_state_str = "1111"
        
        path = []
        current = self.current_state.copy()
        
        # 按正确顺序放回工具：stickP和stickR必须在gripper之前放回
        # 1. 先放回stickP和stickR
        if current[2] == 0:  # stickP被取出
            path.append('stickP2zero')
            current = self.apply_action(current, 'stickP2zero')
        
        if current[3] == 0:  # stickR被取出
            path.append('stickR2zero')
            current = self.apply_action(current, 'stickR2zero')
        
        # 2. 然后放回gripper和frame
        if current[0] == 0:  # gripper被取出
            path.append('gripper2zero')
            current = self.apply_action(current, 'gripper2zero')
        
        if current[1] == 0:  # frame被取出
            path.append('frame2zero')
            current = self.apply_action(current, 'frame2zero')
        
        if not path:
            print("信息: 已经在默认状态")
            return True
        
        print(f"当前状态: {current_state_str}")
        print(f"目标状态: {target_state_str}")
        print(f"执行动作: {' -> '.join(path)}")
        
        # 执行路径中的每个动作
        for action in path:
            self.current_state = self.apply_action(self.current_state, action)
        
        return True
    
    def print_status(self):
        """打印当前状态信息"""
        print(f"\n当前状态: {self.get_current_state_string()}")
        print(f"gripper: {'home' if self.current_state[0] else 'away'} | frame: {'home' if self.current_state[1] else 'away'} | stickP: {'home' if self.current_state[2] else 'away'} | stickR: {'home' if self.current_state[3] else 'away'}")


def main():
    """主函数 - 交互式测试"""
    manager = ToolStateManager()

    print("工具状态切换系统")
    print("=" * 50)
    print("按钮说明:")
    print("A - gripper取反 (位置0取反)")
    print("B - frame取反 (位置1取反)")
    print("C - gripper取出+stickP取反 (位置0置0,位置2取反)")
    print("D - gripper取出+stickR取反 (位置0置0,位置3取反)")
    print("R - 工具归位")
    print("Q - 退出")
    print("=" * 50)

    while True:
        
        manager.print_status()
        
        choice = input("\n请输入按钮 (A/B/C/D/R/Q): ").strip().upper()
        
        if choice == 'Q':
            print("退出程序")
            break
        elif choice == 'R':
            manager.reset_to_default()
        elif choice in ['A', 'B', 'C', 'D']:
            manager.execute_button_action(choice)
        else:
            print("无效输入，请输入 A, B, C, D, R 或 Q")


if __name__ == "__main__":
    main()
