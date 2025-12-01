#!/usr/bin/env python3
"""
Lift Robot Server Positioning Script (HTTP Version)

Simplified script that uses LiftPlatformController from ur_execute_base.py

Workflow:
    Phase 1 - Initialization:
        - Pushrod down (timed)
        - Platform down to base (auto-stop at 832mm)
        - Adjust pushrod to initial_height + offset
    
    Phase 2 - Raise to Server:
        - Platform force_up to contact force threshold
    
    Phase 3 - Apply Final Force:
        - Wait delay period
        - Platform force_up to final force threshold

Run:
    python3 scripts/lift_robot_server_positioning_http.py [options]
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
    p = argparse.ArgumentParser(description='Lift Robot Server Positioning (HTTP)')
    p.add_argument('--pushrod-offset-mm', type=float, default=18.0,
                   help='Pushrod offset in mm above initial height (default: 18.0)')
    p.add_argument('--pushrod-duration', type=float, default=7.0,
                   help='Pushrod down duration in seconds (default: 7.0)')
    p.add_argument('--force-threshold', type=float, default=140.0,
                   help='Force threshold in Newtons for server contact (default: 140.0)')
    p.add_argument('--force-threshold-final', type=float, default=460.0,
                   help='Final force threshold in Newtons (default: 460.0)')
    p.add_argument('--force-delay', type=float, default=10.0,
                   help='Delay in seconds before final force (default: 10.0)')
    p.add_argument('--force-timeout', type=float, default=90.0,
                   help='Timeout for force_up operations (default: 90.0)')
    p.add_argument('--web-base', type=str, default='http://localhost:8090',
                   help='Lift web server base URL (default: http://localhost:8090)')
    return p.parse_args()


def main():
    args = parse_args()
    
    # Initialize lift platform controller
    executor = LiftPlatformController(base_url=args.web_base)
    
    result_data = {
        'initial_height': None,
        'adjusted_height': None,
        'contact_height': None,
        'contact_force': None,
        'final_height': None,
        'final_force': None,
        'force_threshold': args.force_threshold,
        'force_threshold_final': args.force_threshold_final,
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
        
        # Step 1: Pushrod down (timed)
        pushrod_result = executor.lift_pushrod_down_timed(duration=args.pushrod_duration)
        if not pushrod_result['success']:
            raise RuntimeError(f"Pushrod down failed: {pushrod_result.get('error')}")
        
        # Step 2: Platform down to base (auto-stop at 832mm)
        platform_result = executor.lift_platform_down_to_base(timeout=30)
        if not platform_result['success']:
            raise RuntimeError(f"Platform down to base failed: {platform_result.get('error')}")
        
        initial_height = platform_result.get('height')
        if initial_height is None:
            raise RuntimeError("Failed to get initial height")
        
        # Step 3: Calculate and adjust pushrod height
        adjusted_height = initial_height + args.pushrod_offset_mm
        print(f"📊 Initial height: {initial_height:.2f}mm")
        print(f"🎯 Adjusted height: {adjusted_height:.2f}mm (+{args.pushrod_offset_mm:.2f}mm)")
        
        adjust_result = executor.lift_pushrod_goto_height(
            target_height=adjusted_height,
            timeout=30
        )
        if not adjust_result['success']:
            raise RuntimeError(f"Pushrod adjustment failed: {adjust_result.get('error')}")
        
        result_data['initial_height'] = initial_height
        result_data['adjusted_height'] = adjusted_height
        
        print("=" * 60)
        print("✅ INITIALIZATION COMPLETE")
        print("=" * 60)
        
        # ═══════════════════════════════════════════════════════════
        # Phase 2: Raise to Server (Contact Detection)
        # ═══════════════════════════════════════════════════════════
        print("\n" + "=" * 60)
        print(f"PHASE 2: RAISE TO SERVER (Target Force: {args.force_threshold}N)")
        print("=" * 60)
        
        contact_result = executor.lift_platform_force_up(
            target_force=args.force_threshold,
            timeout=args.force_timeout
        )
        
        if not contact_result['success']:
            raise RuntimeError(f"Contact detection failed: {contact_result.get('error')}")
        
        result_data['contact_height'] = contact_result.get('final_height')
        result_data['contact_force'] = contact_result.get('final_force')
        
        # ═══════════════════════════════════════════════════════════
        # Phase 3: Apply Final Force
        # ═══════════════════════════════════════════════════════════
        print("\n" + "=" * 60)
        print(f"PHASE 3: APPLY FINAL FORCE (Delay: {args.force_delay}s, Target: {args.force_threshold_final}N)")
        print("=" * 60)
        
        print(f"⏳ Waiting {args.force_delay:.1f}s before applying final force...")
        import time
        time.sleep(args.force_delay)
        
        final_result = executor.lift_platform_force_up(
            target_force=args.force_threshold_final,
            timeout=args.force_timeout
        )
        
        if not final_result['success']:
            raise RuntimeError(f"Final force application failed: {final_result.get('error')}")
        
        result_data['final_height'] = final_result.get('final_height')
        result_data['final_force'] = final_result.get('final_force')
        result_data['status'] = 'success'
        
        print("\n" + "=" * 60)
        print("✅ ALL PHASES COMPLETED SUCCESSFULLY")
        print("=" * 60)
        
    except Exception as e:
        result_data['error'] = str(e)
        print(f"\n❌ ERROR: {e}", file=sys.stderr)
    
    # Output JSON summary
    print("\n" + json.dumps(result_data, indent=2))


if __name__ == '__main__':
    main()
