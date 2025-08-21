#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
工具状态切换系统
管理四种工具（jaw, holder, stickP, stickR）的状态切换
状态表示：每个位代表一个工具的状态，1表示在原位，0表示被取出
格式：[jaw, holder, stickP, stickR]
"""

class ToolStateManager:
    def __init__(self):
        # 初始状态：所有工具都在原位
        self.current_state = [1, 1, 1, 1]  # [jaw, holder, stickP, stickR]
        
        # 定义目标状态
        self.target_states = {
            'A': [0, 1, 1, 1],  # jaw
            'B': [1, 0, 1, 1],  # holder
            'C': [0, 1, 0, 1],  # jaw + stickP
            'D': [0, 1, 1, 0]   # jaw + stickR
        }
        
        # 定义有效状态（根据您的分析，排除不可能的状态）
        # stickP和stickR只有在jaw被取出时才可以夹取
        self.valid_states = {
            (1, 1, 1, 1),  # 1111 - 默认状态，所有工具在原位
            (0, 1, 1, 1),  # 0111 - 只取jaw
            (1, 0, 1, 1),  # 1011 - 只取holder
            (0, 1, 0, 1),  # 0101 - 取jaw后才可以取stickP
            (0, 1, 1, 0),  # 0110 - 取jaw后才可以取stickR
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
        
        if action_name == 'zero2jaw':
            new_state[0] = 0  # jaw被取出
        elif action_name == 'jaw2zero':
            new_state[0] = 1  # jaw放回
        elif action_name == 'zero2holder':
            new_state[1] = 0  # holder被取出
        elif action_name == 'holder2zero':
            new_state[1] = 1  # holder放回
        elif action_name == 'zero2stickP':
            new_state[2] = 0  # stickP被取出
        elif action_name == 'stickP2zero':
            new_state[2] = 1  # stickP放回
        elif action_name == 'zero2stickR':
            new_state[3] = 0  # stickR被取出
        elif action_name == 'stickR2zero':
            new_state[3] = 1  # stickR放回
        
        return new_state
    
    def find_shortest_path(self, target_button):
        """使用BFS找到从当前状态到目标状态的最短路径"""
        if target_button not in self.target_states:
            return None, f"无效的目标按钮: {target_button}"
        
        target_state = tuple(self.target_states[target_button])
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
            ('zero2jaw', lambda s: (0, s[1], s[2], s[3]) if s[0] == 1 else None),
            ('jaw2zero', lambda s: (1, s[1], s[2], s[3]) if s[0] == 0 else None),
            ('zero2holder', lambda s: (s[0], 0, s[2], s[3]) if s[1] == 1 else None),
            ('holder2zero', lambda s: (s[0], 1, s[2], s[3]) if s[1] == 0 else None),
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
        
        target_state = self.target_states[button]
        print(f"当前状态: {self.get_current_state_string()}")
        print(f"目标状态: {self.state_to_string(target_state)}")
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
        
        # 按正确顺序放回工具：stickP和stickR必须在jaw之前放回
        # 1. 先放回stickP和stickR
        if current[2] == 0:  # stickP被取出
            path.append('stickP2zero')
            current = self.apply_action(current, 'stickP2zero')
        
        if current[3] == 0:  # stickR被取出
            path.append('stickR2zero')
            current = self.apply_action(current, 'stickR2zero')
        
        # 2. 然后放回jaw和holder
        if current[0] == 0:  # jaw被取出
            path.append('jaw2zero')
            current = self.apply_action(current, 'jaw2zero')
        
        if current[1] == 0:  # holder被取出
            path.append('holder2zero')
            current = self.apply_action(current, 'holder2zero')
        
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
        print(f"jaw: {'home' if self.current_state[0] else 'away'} | holder: {'home' if self.current_state[1] else 'away'} | stickP: {'home' if self.current_state[2] else 'away'} | stickR: {'home' if self.current_state[3] else 'away'}")


def main():
    """主函数 - 交互式测试"""
    manager = ToolStateManager()

    print("工具状态切换系统")
    print("=" * 50)
    print("按钮说明:")
    print("A - 取jaw")
    print("B - 取holder")
    print("C - 取stickP")
    print("D - 取stickR")
    print("R - 工具归位")
    print("S - 显示当前状态")
    print("Q - 退出")
    print("=" * 50)

    while True:
        
        manager.print_status()
        
        choice = input("\n请输入按钮 (A/B/C/D/R/S/Q): ").strip().upper()
        
        if choice == 'Q':
            print("退出程序")
            break
        elif choice == 'S':
            continue  # 状态已经在循环开始时显示
        elif choice == 'R':
            manager.reset_to_default()
        elif choice in ['A', 'B', 'C', 'D']:
            manager.execute_button_action(choice)
        else:
            print("无效输入，请输入 A, B, C, D, R, S 或 Q")


if __name__ == "__main__":
    main()
