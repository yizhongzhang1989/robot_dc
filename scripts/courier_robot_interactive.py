#!/usr/bin/env python3
"""
Courier Robot Interactive Mode
Provides command-line interface for manual robot control
"""


def interactive_mode(robot):
    """
    Interactive command-line mode for manual control
    
    Args:
        robot: CourierRobotWebAPI instance
    
    Available commands:
    
    Status & Info:
      status, st        - Show current status
      sensor, sn        - Show sensor data
      help, h, ?        - Show this help
      quit, exit, q     - Exit interactive mode
    
    Server Configuration:
      setid <id>        - Set server ID (updates high_pos/middle_pos/low_pos), e.g., 'setid 15'
    
    Platform Height Control:
      goto <height>     - Go to height (mm), e.g., 'goto 900' (runs in background, can be interrupted)
      g <height>        - Short form of goto
      goto! <height>    - Non-blocking goto (fire-and-forget)
      g! <height>       - Non-blocking short form
    
    Platform Force Control:
      fup <force>       - Force control up (N), e.g., 'fup 50' (background)
      fdown <force>     - Force control down (N), e.g., 'fdown 30' (background)
      fup! <force>      - Non-blocking force up
      fdown! <force>    - Non-blocking force down
      hybrid <h> <f>    - Hybrid control, e.g., 'hybrid 900 50' or 'hybrid middle_pos 200' (background)
      hybrid! <h> <f>   - Non-blocking hybrid control
                          <h> can be: numeric height (mm), 'high_pos', 'high', 'middle_pos', 'middle', 'mid', 'low_pos', or 'low'
    
    Platform Manual Control:
      up                - Manual up (blocking: waits until completed)
      up!               - Manual up (non-blocking: fire-and-forget, use 'stop' to stop)
      down              - Manual down (blocking: waits until completed)
      down!             - Manual down (non-blocking: fire-and-forget, use 'stop' to stop)
      stop              - Stop platform (can interrupt any command!)
    
    Pushrod Control:
      pup               - Pushrod manual up (blocking: waits until completed)
      pup!              - Pushrod manual up (non-blocking: use 'pstop' to stop)
      pdown             - Pushrod manual down (blocking: waits until completed)
      pdown!            - Pushrod manual down (non-blocking: use 'pstop' to stop)
      pgoto <height>    - Pushrod goto absolute height (mm, background)
      pgoto! <height>   - Pushrod goto absolute height (non-blocking)
      prel <offset>     - Pushrod relative move (mm, background), e.g., 'prel 10' or 'prel -5'
      prel! <offset>    - Pushrod relative move (non-blocking)
      pstop             - Stop pushrod (can interrupt any command!)
    
    Emergency:
      reset, emergency  - Emergency reset (can interrupt any command!)
    
    Note: Commands without '!' run in background - you can type 'stop' anytime to interrupt!
    """
    print("\n" + "="*60)
    print("🤖 CourierRobot Interactive Mode")
    print("="*60)
    print("Type 'help' for available commands, 'quit' to exit")
    print("="*60 + "\n")
    
    while True:
        try:
            cmd = input("robot> ").strip().lower()
            
            if not cmd:
                continue
            
            parts = cmd.split()
            command = parts[0]
            
            # Exit commands
            if command in ['quit', 'exit', 'q']:
                print("👋 Exiting interactive mode")
                break
            
            # Help
            elif command in ['help', 'h', '?']:
                print(interactive_mode.__doc__)
            
            # Status commands
            elif command in ['status', 'st']:
                import json
                status = robot.get_status()
                print(json.dumps(status, indent=2))
            
            elif command in ['sensor', 'sn']:
                robot.get_sensor_data()
            
            # Server ID configuration
            elif command == 'setid':
                if len(parts) < 2:
                    print("❌ Usage: setid <id>")
                else:
                    try:
                        server_id = int(parts[1])
                        robot.set_server_id(server_id)
                    except ValueError:
                        print("❌ Invalid server ID value")
            
            # Platform height control
            elif command in ['goto', 'g', 'goto!', 'g!']:
                if len(parts) < 2:
                    print("❌ Usage: goto <height> or goto! <height> (non-blocking)")
                else:
                    try:
                        height = float(parts[1])
                        wait = not command.endswith('!')  # Non-blocking if ends with !
                        if wait:
                            # Blocking mode: execute in background thread
                            robot._execute_in_background(robot.platform_goto_height, height, wait=True)
                        else:
                            # Non-blocking mode: execute directly
                            robot.platform_goto_height(height, wait=False)
                    except ValueError:
                        print("❌ Invalid height value")
            
            # Platform force control
            elif command in ['fup', 'fup!']:
                if len(parts) < 2:
                    print("❌ Usage: fup <force> or fup! <force> (non-blocking)")
                else:
                    try:
                        force = float(parts[1])
                        wait = not command.endswith('!')
                        if wait:
                            # Blocking mode: execute in background thread
                            robot._execute_in_background(robot.platform_force_up, force, wait=True)
                        else:
                            # Non-blocking mode: execute directly
                            robot.platform_force_up(force, wait=False)
                    except ValueError:
                        print("❌ Invalid force value")
            
            elif command in ['fdown', 'fdown!']:
                if len(parts) < 2:
                    print("❌ Usage: fdown <force> or fdown! <force> (non-blocking)")
                else:
                    try:
                        force = float(parts[1])
                        wait = not command.endswith('!')
                        if wait:
                            # Blocking mode: execute in background thread
                            robot._execute_in_background(robot.platform_force_down, force, wait=True)
                        else:
                            # Non-blocking mode: execute directly
                            robot.platform_force_down(force, wait=False)
                    except ValueError:
                        print("❌ Invalid force value")
            
            elif command in ['hybrid', 'hybrid!']:
                if len(parts) < 3:
                    print("❌ Usage: hybrid <height|high_pos|middle_pos|low_pos> <force> or hybrid! <height|high_pos|middle_pos|low_pos> <force> (non-blocking)")
                else:
                    try:
                        # height can be numeric or string ('high_pos', 'middle_pos', 'low_pos')
                        height_str = parts[1]
                        if height_str.lower() in ['high_pos', 'high', 'middle_pos', 'middle', 'mid', 'low_pos', 'low']:
                            height = height_str  # Pass string directly
                        else:
                            height = float(height_str)  # Parse as numeric
                        
                        force = float(parts[2])
                        wait = not command.endswith('!')
                        if wait:
                            # Blocking mode: execute in background thread
                            robot._execute_in_background(robot.platform_hybrid_control, height, force, wait=True)
                        else:
                            # Non-blocking mode: execute directly
                            robot.platform_hybrid_control(height, force, wait=False)
                    except ValueError:
                        print("❌ Invalid force value")
            
            # Platform manual control
            elif command in ['up', 'up!']:
                blocking = (command == 'up')  # 'up' = blocking, 'up!' = non-blocking
                robot.platform_up(blocking=blocking)
            
            elif command in ['down', 'down!']:
                blocking = (command == 'down')  # 'down' = blocking, 'down!' = non-blocking
                robot.platform_down(blocking=blocking)
            
            elif command == 'stop':
                robot.platform_stop()
            
            # Pushrod control
            elif command in ['pup', 'pup!']:
                blocking = (command == 'pup')  # 'pup' = blocking, 'pup!' = non-blocking
                robot.pushrod_up(blocking=blocking)
            
            elif command in ['pdown', 'pdown!']:
                blocking = (command == 'pdown')  # 'pdown' = blocking, 'pdown!' = non-blocking
                robot.pushrod_down(blocking=blocking)
            
            elif command in ['pgoto', 'pgoto!']:
                if len(parts) < 2:
                    print("❌ Usage: pgoto <height> or pgoto! <height> (non-blocking)")
                else:
                    try:
                        height = float(parts[1])
                        wait = not command.endswith('!')
                        if wait:
                            # Blocking mode: execute in background thread
                            robot._execute_in_background(robot.pushrod_goto_height, height, mode='absolute', wait=True)
                        else:
                            # Non-blocking mode: execute directly
                            robot.pushrod_goto_height(height, mode='absolute', wait=False)
                    except ValueError:
                        print("❌ Invalid height value")
            
            elif command in ['prel', 'prel!']:
                if len(parts) < 2:
                    print("❌ Usage: prel <offset> or prel! <offset> (non-blocking)")
                else:
                    try:
                        offset = float(parts[1])
                        wait = not command.endswith('!')
                        if wait:
                            # Blocking mode: execute in background thread
                            robot._execute_in_background(robot.pushrod_goto_height, offset, mode='relative', wait=True)
                        else:
                            # Non-blocking mode: execute directly
                            robot.pushrod_goto_height(offset, mode='relative', wait=False)
                    except ValueError:
                        print("❌ Invalid offset value")
            
            elif command == 'pstop':
                robot.pushrod_stop()
            
            # Emergency
            elif command in ['reset', 'emergency']:
                robot.emergency_reset()
            
            else:
                print(f"❌ Unknown command: '{command}'. Type 'help' for available commands.")
        
        except KeyboardInterrupt:
            print("\n👋 Exiting interactive mode (Ctrl+C)")
            break
        except Exception as e:
            print(f"❌ Error: {e}")
