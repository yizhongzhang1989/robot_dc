#!/usr/bin/env python3
"""
Lift Robot Server Insert Script (HTTP Version)

Simplified script that uses LiftPlatformController from ur_execute_base.py

Workflow:
    Phase 1 - Initialization:
        - Pushrod down (timed 7s)
        - Platform down to base (auto-stop at 832mm)
        - Platform up to 919mm
        - Pushrod up to 937mm
        - Wait 10s delay
    
    Phase 2 - Force-Controlled Descent:
        - Platform force_down until force < 250N

Run:
    python3 scripts/lift_robot_server_insert_http.py [options]
"""
import json
import sys
import argparse
import os
import time

# Add parent directory to path for imports
try:
    from common.workspace_utils import get_scripts_directory
    scripts_dir = get_scripts_directory()
    if scripts_dir:
        sys.path.insert(0, scripts_dir)
    else:
        sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
except ImportError:
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

# Import only the lift controller (no robot dependencies)
from ur_execute_base import LiftPlatformController


def parse_args():
    p = argparse.ArgumentParser(description='Lift Robot Server Insert (HTTP)')
    p.add_argument('--pushrod-duration', type=float, default=7.0,
                   help='Pushrod down duration in seconds (default: 7.0)')
    p.add_argument('--platform-height', type=float, default=919.0,
                   help='Platform target height in mm (default: 919.0)')
    p.add_argument('--pushrod-height', type=float, default=937.0,
                   help='Pushrod target height in mm (default: 937.0)')
    p.add_argument('--init-delay', type=float, default=10.0,
                   help='Delay in seconds after initialization (default: 10.0)')
    p.add_argument('--force-threshold', type=float, default=250.0,
                   help='Force threshold in Newtons for insert completion (default: 250.0)')
    p.add_argument('--force-timeout', type=float, default=90.0,
                   help='Timeout for force_down operation (default: 90.0)')
    p.add_argument('--web-base', type=str, default='http://localhost:8090',
                   help='Lift web server base URL (default: http://localhost:8090)')
    return p.parse_args()


def main():
    args = parse_args()
    
    # Initialize lift platform controller
    executor = LiftPlatformController(base_url=args.web_base)
    
    result_data = {
        'platform_height': args.platform_height,
        'pushrod_height': args.pushrod_height,
        'final_height': None,
        'final_force': None,
        'force_threshold': args.force_threshold,
        'status': 'error',
        'error': None
    }
    
    try:
        # ═══════════════════════════════════════════════════════════
        # Phase 1: Initialization
        # ═══════════════════════════════════════════════════════════
        print("\n" + "=" * 60)
        print("PHASE 1: INITIALIZATION")
        print("=" * 60)
        
        # Step 1: Pushrod down (timed 7s)
        pushrod_result = executor.lift_pushrod_down_timed(duration=args.pushrod_duration)
        if not pushrod_result['success']:
            raise RuntimeError(f"Pushrod down failed: {pushrod_result.get('error')}")
        
        # Step 2: Platform down to base (auto-stop at 832mm)
        platform_result = executor.lift_platform_down_to_base(timeout=30)
        if not platform_result['success']:
            raise RuntimeError(f"Platform down to base failed: {platform_result.get('error')}")
        
        # Step 3: Platform up to 919mm
        platform_up_result = executor.lift_platform_goto_height(
            target_height=args.platform_height,
            timeout=60
        )
        if not platform_up_result['success']:
            raise RuntimeError(f"Platform goto height failed: {platform_up_result.get('error')}")
        
        # Step 4: Pushrod up to 937mm
        pushrod_up_result = executor.lift_pushrod_goto_height(
            target_height=args.pushrod_height,
            timeout=30
        )
        if not pushrod_up_result['success']:
            raise RuntimeError(f"Pushrod goto height failed: {pushrod_up_result.get('error')}")
        
        # Step 5: Wait delay period
        print(f"⏳ Waiting {args.init_delay:.1f}s...")
        time.sleep(args.init_delay)
        
        print("=" * 60)
        print("✅ INITIALIZATION COMPLETE")
        print("=" * 60)
        
        # ═══════════════════════════════════════════════════════════
        # Phase 2: Force-Controlled Descent
        # ═══════════════════════════════════════════════════════════
        print("\n" + "=" * 60)
        print(f"PHASE 2: FORCE-CONTROLLED DESCENT (Target Force: {args.force_threshold}N)")
        print("=" * 60)
        
        insert_result = executor.lift_platform_force_down(
            target_force=args.force_threshold,
            timeout=args.force_timeout
        )
        
        if not insert_result['success']:
            raise RuntimeError(f"Force-controlled descent failed: {insert_result.get('error')}")
        
        result_data['final_height'] = insert_result.get('final_height')
        result_data['final_force'] = insert_result.get('final_force')
        result_data['status'] = 'success'
        
        print("\n" + "=" * 60)
        print("✅ SERVER INSERT COMPLETED SUCCESSFULLY")
        print("=" * 60)
        
    except Exception as e:
        result_data['error'] = str(e)
        print(f"\n❌ ERROR: {e}", file=sys.stderr)
    
    # Output JSON summary
    print("\n" + json.dumps(result_data, indent=2))


if __name__ == '__main__':
    main()
