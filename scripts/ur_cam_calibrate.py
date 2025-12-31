#!/usr/bin/env python3
"""
UR15 Eye-in-Hand Camera Calibration Script
==========================================

This script performs eye-in-hand calibration for the UR15 robot's end-effector camera
using ChArUco board pattern.

Features:
- Supports ChArUco board calibration pattern
- Reads images and robot poses from temp/ur15_cam_calibration_data/
- Performs intrinsic calibration
- Performs eye-in-hand (hand-eye) calibration
- Generates detailed calibration reports
- Saves results to temp/ur15_calibration_result/

Usage:
    python3 ur_cam_calibetra.py [options]

Options:
    --data-dir DIR          Directory containing calibration data (default: ../temp/ur15_cam_calibration_data)
    --output-dir DIR        Directory to save results (default: ../temp/ur15_calibration_result)
    --config-file FILE      Path to chessboard_config.json (default: ../temp/ur15_cam_calibration_data/chessboard_config.json)
    --min-images INT        Minimum number of valid images required (default: 5)
    --verbose              Enable verbose output
"""

import sys
import os
import argparse
import json
import glob
import numpy as np
import cv2
from datetime import datetime

# Add ThirdParty path
sys.path.append(os.path.join(os.path.dirname(__file__), 'ThirdParty'))

try:
    from camera_calibration_toolkit.core.eye_in_hand_calibration import EyeInHandCalibrator
    from camera_calibration_toolkit.core.intrinsic_calibration import IntrinsicCalibrator
    from camera_calibration_toolkit.core.calibration_patterns import load_pattern_from_json
except ImportError as e:
    print(f"❌ Error importing camera_calibration_toolkit: {e}")
    print("Please ensure the camera_calibration_toolkit is properly installed.")
    sys.exit(1)


def parse_arguments():
    """Parse command line arguments."""
    parser = argparse.ArgumentParser(
        description="UR15 Eye-in-Hand Camera Calibration using ChArUco Board",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    
    parser.add_argument(
        '--data-dir',
        type=str,
        default='../temp/ur15_cam_calibration_data',
        help='Directory containing calibration images and robot poses'
    )
    
    parser.add_argument(
        '--output-dir',
        type=str,
        default='../temp/ur15_cam_calibration_result',
        help='Directory to save calibration results'
    )
    
    parser.add_argument(
        '--config-file',
        type=str,
        default='../temp/ur15_cam_calibration_data/chessboard_config.json',
        help='Path to chessboard_config.json file'
    )
    
    parser.add_argument(
        '--min-images',
        type=int,
        default=5,
        help='Minimum number of valid images required'
    )
    
    parser.add_argument(
        '--verbose',
        action='store_true',
        help='Enable verbose output'
    )
    
    return parser.parse_args()


def load_pattern_from_config(config_file, verbose=False):
    """
    Load and create calibration pattern from JSON configuration file.
    
    Args:
        config_file: Path to chessboard_config.json file
        verbose: Whether to print loading information
        
    Returns:
        Calibration pattern object created from JSON config
    """
    if not os.path.exists(config_file):
        raise FileNotFoundError(f"Configuration file not found: {config_file}")
    
    if verbose:
        print(f"\n📋 Loading pattern configuration from: {config_file}")
    
    with open(config_file, 'r') as f:
        config_data = json.load(f)
    
    # Load pattern directly from JSON using the toolkit's loader
    pattern = load_pattern_from_json(config_data)
    
    if verbose:
        print(f"   Pattern: {config_data.get('name', 'Unknown')}")
        print(f"   Pattern ID: {config_data.get('pattern_id', 'Unknown')}")
        if 'parameters' in config_data:
            params = config_data['parameters']
            if 'width' in params and 'height' in params:
                print(f"   Board size: {params['width']}×{params['height']} squares")
            if 'square_size' in params:
                print(f"   Square size: {params['square_size']}m ({params['square_size']*1000:.1f}mm)")
            if 'marker_size' in params:
                print(f"   Marker size: {params['marker_size']}m ({params['marker_size']*1000:.1f}mm)")
    
    return pattern


def load_calibration_data(data_dir, verbose=False):
    """
    Load images and robot poses from calibration data directory.
    
    Args:
        data_dir: Directory containing .jpg images and .json pose files
        verbose: Whether to print detailed loading information
        
    Returns:
        Tuple of (images, end2base_matrices, image_filenames)
    """
    if not os.path.exists(data_dir):
        raise FileNotFoundError(f"Data directory not found: {data_dir}")
    
    # Find all image files
    image_files = sorted(glob.glob(os.path.join(data_dir, '*.jpg')))
    
    if len(image_files) == 0:
        raise ValueError(f"No .jpg images found in {data_dir}")
    
    if verbose:
        print(f"📁 Loading calibration data from: {data_dir}")
        print(f"   Found {len(image_files)} image files")
    
    images = []
    end2base_matrices = []
    image_filenames = []
    
    for img_file in image_files:
        # Load image
        img = cv2.imread(img_file)
        if img is None:
            if verbose:
                print(f"   ⚠️  Failed to load image: {img_file}")
            continue
        
        # Find corresponding JSON file
        img_basename = os.path.basename(img_file).replace('.jpg', '')
        json_file = os.path.join(data_dir, f'{img_basename}.json')
        
        if not os.path.exists(json_file):
            if verbose:
                print(f"   ⚠️  No pose file found for: {img_file}")
            continue
        
        # Load robot pose
        try:
            with open(json_file, 'r') as f:
                pose_data = json.load(f)
            
            # Extract end2base transformation matrix
            if 'end2base' not in pose_data:
                if verbose:
                    print(f"   ⚠️  No 'end2base' field in: {json_file}")
                continue
            
            end2base_matrix = np.array(pose_data['end2base'], dtype=np.float64)
            
            if end2base_matrix.shape != (4, 4):
                if verbose:
                    print(f"   ⚠️  Invalid end2base matrix shape in: {json_file}")
                continue
            
            # Successfully loaded image and pose
            images.append(img)
            end2base_matrices.append(end2base_matrix)
            image_filenames.append(os.path.basename(img_file))
            
            if verbose:
                print(f"   ✅ Loaded: {img_basename}")
                
        except Exception as e:
            if verbose:
                print(f"   ⚠️  Error loading pose from {json_file}: {e}")
            continue
    
    if verbose:
        print(f"   Successfully loaded {len(images)} image-pose pairs")
    
    return images, end2base_matrices, image_filenames





def perform_intrinsic_calibration(images, pattern, verbose=False):
    """
    Perform camera intrinsic calibration.
    
    Args:
        images: List of calibration images
        pattern: Calibration pattern object
        verbose: Whether to print calibration progress
        
    Returns:
        Tuple of (results dictionary, intrinsic_calibrator object)
    """
    if verbose:
        print(f"\n🔧 Step 1: Intrinsic Calibration")
        print(f"   Processing {len(images)} images...")
    
    try:
        intrinsic_calibrator = IntrinsicCalibrator(
            images=images,
            calibration_pattern=pattern
        )
        
        results = intrinsic_calibrator.calibrate(verbose=verbose, flags=cv2.CALIB_RATIONAL_MODEL)
        
        if results is None:
            raise ValueError("Intrinsic calibration failed")
        
        if verbose:
            print(f"   ✅ Intrinsic calibration completed")
            print(f"   RMS Error: {results['rms_error']:.4f} pixels")
            print(f"   Valid images: {results.get('valid_image_count', 'N/A')}")
        
        return results, intrinsic_calibrator
        
    except Exception as e:
        print(f"❌ Intrinsic calibration failed: {e}")
        raise


def perform_eye_in_hand_calibration(images, end2base_matrices, pattern, 
                                    camera_matrix, distortion_coeffs, verbose=False):
    """
    Perform eye-in-hand calibration.
    
    Args:
        images: List of calibration images
        end2base_matrices: List of end-effector to base transformation matrices
        pattern: Calibration pattern object
        camera_matrix: Camera intrinsic matrix
        distortion_coeffs: Camera distortion coefficients
        verbose: Whether to print calibration progress
        
    Returns:
        Dictionary containing calibration results
    """
    if verbose:
        print(f"\n🤖 Step 2: Eye-in-Hand Calibration")
        print(f"   Using {len(images)} image-pose pairs...")
    
    try:
        calibrator = EyeInHandCalibrator(
            images=images,
            end2base_matrices=end2base_matrices,
            calibration_pattern=pattern,
            camera_matrix=camera_matrix,
            distortion_coefficients=distortion_coeffs,
            verbose=verbose
        )
        
        # Perform calibration (will test all methods and select the best)
        result = calibrator.calibrate(verbose=verbose)
        
        if result is None:
            raise ValueError("Eye-in-hand calibration failed")
        
        if verbose:
            print(f"   ✅ Eye-in-hand calibration completed")
            print(f"   RMS Error: {result['rms_error']:.4f} pixels")
            print(f"   Method: {result.get('method_name', 'Unknown')}")
        
        return result, calibrator
        
    except Exception as e:
        print(f"❌ Eye-in-hand calibration failed: {e}")
        raise


def save_calibration_results(output_dir, intrinsic_results, intrinsic_calibrator, 
                            eye_in_hand_result, eye_in_hand_calibrator, 
                            image_filenames, pattern_params, verbose=False):
    """
    Save calibration results to output directory.
    
    Args:
        output_dir: Directory to save results (e.g., temp/ur15_calibration_result)
        intrinsic_results: Intrinsic calibration results
        intrinsic_calibrator: IntrinsicCalibrator object
        eye_in_hand_result: Eye-in-hand calibration results
        eye_in_hand_calibrator: EyeInHandCalibrator object
        image_filenames: List of image filenames used
        pattern_params: Pattern parameters dictionary
        verbose: Whether to print save information
    """
    if verbose:
        print(f"\n💾 Saving calibration results")
    
    # 1. Generate intrinsic calibration report (like web interface does)
    intrinsic_report_dir = os.path.join(output_dir, 'ur15_intrinsic_calibration_report')
    try:
        intrinsic_report_result = intrinsic_calibrator.generate_calibration_report(intrinsic_report_dir)
        
        if intrinsic_report_result and verbose:
            print(f"   ✅ Generated intrinsic report: {intrinsic_report_dir}/")
            if 'html_report' in intrinsic_report_result:
                print(f"      - calibration_report.html")
            if 'json_data' in intrinsic_report_result:
                print(f"      - calibration_data.json")
    except Exception as e:
        if verbose:
            print(f"   ⚠️  Failed to generate intrinsic calibration report: {e}")
    
    # 2. Generate eye-in-hand calibration report (like web interface does)
    eye_in_hand_report_dir = os.path.join(output_dir, 'ur15_eye_in_hand_calibration_report')
    try:
        eye_in_hand_report_result = eye_in_hand_calibrator.generate_calibration_report(eye_in_hand_report_dir)
        
        if eye_in_hand_report_result and verbose:
            print(f"   ✅ Generated eye-in-hand report: {eye_in_hand_report_dir}/")
            if 'html_report' in eye_in_hand_report_result:
                print(f"      - calibration_report.html")
            if 'json_data' in eye_in_hand_report_result:
                print(f"      - calibration_data.json")
    except Exception as e:
        if verbose:
            print(f"   ⚠️  Failed to generate eye-in-hand calibration report: {e}")
    
    # 3. Save calibration_result.json in ur15_camera_parameters directory (like web interface)
    camera_params_dir = os.path.join(output_dir, 'ur15_camera_parameters')
    os.makedirs(camera_params_dir, exist_ok=True)
    
    # Save intrinsic calibration result (matching temp/camera_parameters/calibration_result.json format)
    intrinsic_output_data = {
        'success': True,
        'rms_error': float(intrinsic_results['rms_error']),
        'camera_matrix': intrinsic_results['camera_matrix'].tolist(),
        'distortion_coefficients': intrinsic_results['distortion_coefficients'].flatten().tolist(),
        'image_count': len(image_filenames),
        'report_html': os.path.join(intrinsic_report_dir, 'calibration_report.html') if intrinsic_report_result and os.path.exists(os.path.join(intrinsic_report_dir, 'calibration_report.html')) else '',
        'report_json': os.path.join(intrinsic_report_dir, 'calibration_data.json') if intrinsic_report_result and os.path.exists(os.path.join(intrinsic_report_dir, 'calibration_data.json')) else ''
    }
    
    intrinsic_output_path = os.path.join(camera_params_dir, 'ur15_cam_calibration_result.json')
    with open(intrinsic_output_path, 'w') as f:
        json.dump(intrinsic_output_data, f, indent=2)
    
    if verbose:
        print(f"   ✅ Saved: {camera_params_dir}/ur15_cam_calibration_result.json")
    
    # 4. Save eye_in_hand_result.json in ur15_camera_parameters directory (like web interface)
    eye_in_hand_output_data = {
        'success': True,
        'rms_error': float(eye_in_hand_result['rms_error']),
        'intrinsic_rms_error': float(intrinsic_results['rms_error']),
        'cam2end_matrix': eye_in_hand_result['cam2end_matrix'].tolist(),
        'target2base_matrix': eye_in_hand_result['target2base_matrix'].tolist(),
        'camera_matrix': intrinsic_results['camera_matrix'].tolist(),
        'distortion_coefficients': intrinsic_results['distortion_coefficients'].flatten().tolist(),
        'image_count': len(image_filenames),
        'pose_count': len(image_filenames),
        'report_html': os.path.join(eye_in_hand_report_dir, 'calibration_report.html') if eye_in_hand_report_result and os.path.exists(os.path.join(eye_in_hand_report_dir, 'calibration_report.html')) else '',
        'report_json': os.path.join(eye_in_hand_report_dir, 'calibration_data.json') if eye_in_hand_report_result and os.path.exists(os.path.join(eye_in_hand_report_dir, 'calibration_data.json')) else ''
    }
    
    eye_in_hand_output_path = os.path.join(camera_params_dir, 'ur15_cam_eye_in_hand_result.json')
    with open(eye_in_hand_output_path, 'w') as f:
        json.dump(eye_in_hand_output_data, f, indent=2)
    
    if verbose:
        print(f"   ✅ Saved: {camera_params_dir}/ur15_cam_eye_in_hand_result.json")


def print_summary(intrinsic_results, eye_in_hand_result, output_dir):
    """Print calibration summary."""
    print("\n" + "="*70)
    print("🎉 UR15 EYE-IN-HAND CALIBRATION COMPLETED SUCCESSFULLY!")
    print("="*70)
    
    print("\n📊 CALIBRATION RESULTS:")
    print("-"*70)
    
    print("\n🔧 Intrinsic Calibration:")
    print(f"   RMS Error: {intrinsic_results['rms_error']:.4f} pixels")
    print(f"   Valid Images: {intrinsic_results.get('valid_image_count', 'N/A')}")
    
    print("\n🤖 Eye-in-Hand Calibration:")
    print(f"   RMS Error: {eye_in_hand_result['rms_error']:.4f} pixels")
    print(f"   Method: {eye_in_hand_result.get('method_name', 'Unknown')}")
    print(f"   Valid Images: {eye_in_hand_result.get('valid_images', 'N/A')}")
    
    if 'before_opt' in eye_in_hand_result:
        improvement = eye_in_hand_result['before_opt']['rms_error'] - eye_in_hand_result['rms_error']
        improvement_pct = (improvement / eye_in_hand_result['before_opt']['rms_error']) * 100
        print(f"\n✨ Optimization:")
        print(f"   Initial Error: {eye_in_hand_result['before_opt']['rms_error']:.4f} pixels")
        print(f"   Final Error: {eye_in_hand_result['rms_error']:.4f} pixels")
        print(f"   Improvement: {improvement:.4f} pixels ({improvement_pct:.1f}%)")
    
    print(f"\n📁 Results saved to: {output_dir}/")
    print("   ├── ur15_intrinsic_calibration_report/")
    print("   │   ├── calibration_report.html   (Intrinsic calibration HTML report)")
    print("   │   ├── calibration_data.json     (Intrinsic calibration data)")
    print("   │   ├── original_images/          (Original calibration images)")
    print("   │   ├── pattern_detection/        (Pattern detection visualization)")
    print("   │   ├── reprojection/             (Reprojection error visualization)")
    print("   │   ├── undistorted/              (Undistorted images)")
    print("   │   └── analysis/                 (Analysis charts)")
    print("   ├── ur15_eye_in_hand_calibration_report/")
    print("   │   ├── calibration_report.html   (Eye-in-hand calibration HTML report)")
    print("   │   ├── calibration_data.json     (Eye-in-hand calibration data)")
    print("   │   ├── original_images/          (Original calibration images)")
    print("   │   ├── pattern_detection/        (Pattern detection visualization)")
    print("   │   ├── reprojection/             (Reprojection error visualization)")
    print("   │   ├── undistorted/              (Undistorted images)")
    print("   │   └── analysis/                 (Analysis charts)")
    print("   └── ur15_camera_parameters/")
    print("       ├── ur15_cam_calibration_result.json        (Intrinsic calibration results)")
    print("       └── ur15_cam_eye_in_hand_result.json        (Eye-in-hand calibration results)")
    
    print("\n" + "="*70)
    print("✅ Calibration completed! You can now use the results for your application.")
    print("="*70 + "\n")


def main():
    """Main calibration function."""
    args = parse_arguments()
    
    print("="*70)
    print("🤖 UR15 EYE-IN-HAND CAMERA CALIBRATION")
    print("="*70)
    print(f"Data directory: {args.data_dir}")
    print(f"Output directory: {args.output_dir}")
    print("="*70)
    
    try:
        # Use the config file path from arguments (already has default value)
        config_file = args.config_file
        
        # Load pattern directly from JSON config (correct method)
        calibration_pattern = load_pattern_from_config(config_file, verbose=args.verbose)
        
        # Load calibration data
        images, end2base_matrices, image_filenames = load_calibration_data(
            args.data_dir, verbose=args.verbose
        )
        
        if len(images) < args.min_images:
            print(f"\n❌ ERROR: Insufficient calibration data!")
            print(f"   Found: {len(images)} valid image-pose pairs")
            print(f"   Required: {args.min_images} minimum")
            print(f"   Please capture more calibration images.")
            return 1
        
        # Perform intrinsic calibration
        intrinsic_results, intrinsic_calibrator = perform_intrinsic_calibration(
            images, calibration_pattern, verbose=args.verbose
        )
        
        # Perform eye-in-hand calibration
        eye_in_hand_result, eye_in_hand_calibrator = perform_eye_in_hand_calibration(
            images,
            end2base_matrices,
            calibration_pattern,
            intrinsic_results['camera_matrix'],
            intrinsic_results['distortion_coefficients'],
            verbose=args.verbose
        )
        
        # Save results
        save_calibration_results(
            args.output_dir,
            intrinsic_results,
            intrinsic_calibrator,
            eye_in_hand_result,
            eye_in_hand_calibrator,
            image_filenames,
            None,  # pattern_params no longer used
            verbose=args.verbose
        )
        
        # Print summary
        print_summary(intrinsic_results, eye_in_hand_result, args.output_dir)
        
        return 0
        
    except FileNotFoundError as e:
        print(f"\n❌ ERROR: {e}")
        print(f"Please check that the data directory exists and contains calibration data.")
        return 1
        
    except ValueError as e:
        print(f"\n❌ ERROR: {e}")
        return 1
        
    except Exception as e:
        print(f"\n❌ UNEXPECTED ERROR: {e}")
        import traceback
        traceback.print_exc()
        return 1


if __name__ == "__main__":
    sys.exit(main())
