#!/usr/bin/env python3
"""
Keypoint Tracking Task
=====================

This script implements keypoint tracking functionality using FFPPWebAPIKeypointTracker directly.
It loads reference images and keypoints from the positioning_data folder,
then tracks keypoints across target images (1-5.jpg).

Dependencies:
- FFPPWebAPIKeypointTracker: For keypoint tracking via FlowFormer++ web service
- json: For data handling
- os: For file operations
- cv2: For image handling
- PIL: For image processing
"""

import os
import sys
import json
import time
import logging
import cv2
import numpy as np
from typing import Dict, List, Optional
import traceback
from PIL import Image

# Add common package to path for workspace utilities
current_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.dirname(current_dir)
common_path = os.path.join(project_root, 'colcon_ws/src/common')
sys.path.insert(0, common_path)

# Add robot_vision path for FFPPWebAPIKeypointTracker
robot_vision_path = os.path.join(current_dir, 'ThirdParty', 'robot_vision')
sys.path.insert(0, robot_vision_path)

# Import workspace utilities
try:
    from common import get_workspace_root, get_temp_directory
    COMMON_AVAILABLE = True
except ImportError as e:
    print(f"Warning: Could not import common package: {e}")
    COMMON_AVAILABLE = False

# Import FFPPWebAPIKeypointTracker
try:
    from core.ffpp_webapi_keypoint_tracker import FFPPWebAPIKeypointTracker
    FFPP_WEBAPI_AVAILABLE = True
except ImportError as e:
    print(f"Warning: Could not import FFPPWebAPIKeypointTracker: {e}")
    print("Make sure you're in the correct environment and robot_vision is available.")
    FFPP_WEBAPI_AVAILABLE = False

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

class KeypointTrackerInTaskEstimation:
    """Keypoint tracking task using FFPPWebAPIKeypointTracker directly."""
    
    def __init__(self, service_url: str = "http://msraig-ubuntu-3:8001"):
        """Initialize the keypoint tracker.
        
        Args:
            service_url: Base URL for the FlowFormer++ web service
        """
        if not FFPP_WEBAPI_AVAILABLE:
            raise ImportError("FFPPWebAPIKeypointTracker is not available")
            
        self.service_url = service_url.rstrip('/')
        
        # Initialize FFPPWebAPIKeypointTracker
        try:
            self.tracker = FFPPWebAPIKeypointTracker(
                service_url=service_url,
                timeout=30,
                image_format="jpg",
                jpeg_quality=95
            )
        except Exception as e:
            logger.error(f"Failed to initialize FFPPWebAPIKeypointTracker: {e}")
            raise
        
        # Setup data paths using workspace utilities if available
        if COMMON_AVAILABLE:
            try:
                temp_dir = get_temp_directory()
                self.positioning_data_path = os.path.join(temp_dir, "positioning_data")
                self.visualization_dir = os.path.join(temp_dir, "keypoints_track_results")
                logger.info(f"Using workspace utilities - temp directory: {temp_dir}")
            except Exception as e:
                logger.warning(f"Could not use workspace utilities: {e}")
                # Fallback to hardcoded paths
                self.positioning_data_path = "/home/a/Documents/robot_dc/temp/positioning_data"
                self.visualization_dir = "/home/a/Documents/robot_dc/temp/keypoints_track_results"
        else:
            # Fallback to hardcoded paths
            self.positioning_data_path = "/home/a/Documents/robot_dc/temp/positioning_data"
            self.visualization_dir = "/home/a/Documents/robot_dc/temp/keypoints_track_results"
        
        # Setup file paths
        self.ref_img_path = os.path.join(self.positioning_data_path, "ref_img.jpg")
        self.ref_keypoints_path = os.path.join(self.positioning_data_path, "ref_keypoints.json")
        
        # Create visualization directory
        if not os.path.exists(self.visualization_dir):
            os.makedirs(self.visualization_dir)
            logger.info(f"Created visualization directory: {self.visualization_dir}")
        
        logger.info(f"Initialized KeypointTrackerInTaskEstimation with service URL: {self.service_url}")
        logger.info(f"Positioning data path: {self.positioning_data_path}")
        logger.info(f"Visualization results path: {self.visualization_dir}")
        logger.info(f"Using FFPPWebAPIKeypointTracker directly (no HTTP client needed)")
    
    def check_api_health(self) -> bool:
        """Check if the FlowFormer++ web service is healthy and ready.
        
        Returns:
            bool: True if service is healthy, False otherwise
        """
        try:
            health_result = self.tracker.get_service_health()
            is_healthy = health_result.get('success', False)
            logger.info(f"Service health check: {'✅ Healthy' if is_healthy else '❌ Degraded'}")
            
            if is_healthy:
                logger.info(f"Service status: {health_result.get('status', 'unknown')}")
                logger.info(f"Message: {health_result.get('message', 'No message')}")
            else:
                logger.error(f"Health check failed: {health_result.get('error', 'Unknown error')}")
                
            return is_healthy
        except Exception as e:
            logger.error(f"Failed to check service health: {str(e)}")
            return False
    
    def load_reference_keypoints(self) -> List[Dict]:
        """Load reference keypoints from JSON file.
        
        Returns:
            List[Dict]: List of keypoint dictionaries with x, y coordinates
        """
        try:
            if not os.path.exists(self.ref_keypoints_path):
                raise FileNotFoundError(f"Reference keypoints file not found: {self.ref_keypoints_path}")
            
            with open(self.ref_keypoints_path, 'r') as f:
                data = json.load(f)
            
            # Extract keypoints and convert to the format expected by API
            keypoints = []
            for kp in data.get('keypoints', []):
                keypoints.append({
                    'x': float(kp['x']),
                    'y': float(kp['y']),
                    'id': kp.get('id'),
                    'label': kp.get('name', f"Point {kp.get('id', len(keypoints) + 1)}")
                })
            
            logger.info(f"Loaded {len(keypoints)} reference keypoints")
            return keypoints
            
        except Exception as e:
            logger.error(f"Failed to load reference keypoints: {str(e)}")
            raise
    
    def set_reference_image(self, keypoints: List[Dict]) -> bool:
        """Set reference image with keypoints using FFPPWebAPIKeypointTracker.
        
        Args:
            keypoints: List of keypoint dictionaries
            
        Returns:
            bool: True if successful, False otherwise
        """
        try:
            if not os.path.exists(self.ref_img_path):
                raise FileNotFoundError(f"Reference image not found: {self.ref_img_path}")
            
            # Load reference image
            ref_image = cv2.imread(self.ref_img_path)
            if ref_image is None:
                raise ValueError(f"Failed to load reference image: {self.ref_img_path}")
            
            # Convert BGR to RGB for the tracker
            ref_image_rgb = cv2.cvtColor(ref_image, cv2.COLOR_BGR2RGB)
            
            # Set reference image with keypoints
            result = self.tracker.set_reference_image(
                image=ref_image_rgb,
                keypoints=keypoints,
                image_name='reference'
            )
            
            if result.get('success', False):
                logger.info(f"✅ Reference image set successfully with {len(keypoints)} keypoints")
                logger.info(f"Reference key: {result.get('key', 'unknown')}")
                logger.info(f"Keypoints count: {result.get('keypoints_count', 0)}")
                return True
            else:
                logger.error(f"Failed to set reference image: {result.get('error', 'Unknown error')}")
                return False
                
        except Exception as e:
            logger.error(f"Error setting reference image: {str(e)}")
            logger.error(traceback.format_exc())
            return False
    
    def track_keypoints_in_image(self, target_image_path: str, bidirectional: bool = False) -> Optional[Dict]:
        """Track keypoints in a target image.
        
        Args:
            target_image_path: Path to the target image
            bidirectional: Whether to enable bidirectional validation
            
        Returns:
            Dict: Tracking result or None if failed
        """
        try:
            if not os.path.exists(target_image_path):
                raise FileNotFoundError(f"Target image not found: {target_image_path}")
            
            # Load target image
            target_image = cv2.imread(target_image_path)
            if target_image is None:
                raise ValueError(f"Failed to load target image: {target_image_path}")
            
            # Convert BGR to RGB for the tracker
            target_image_rgb = cv2.cvtColor(target_image, cv2.COLOR_BGR2RGB)
            
            # Track keypoints
            result = self.tracker.track_keypoints(
                target_image=target_image_rgb,
                reference_name='reference',
                bidirectional=bidirectional,
                return_flow=False
            )
            
            if result.get('success', False):
                logger.info(f"✅ Tracked {result.get('keypoints_count', 0)} keypoints in {os.path.basename(target_image_path)}")
                logger.info(f"Processing time: {result.get('total_processing_time', 0):.3f}s")
                
                # Format result to match expected structure
                tracking_data = {
                    'tracked_keypoints': result.get('tracked_keypoints', []),
                    'keypoints_count': result.get('keypoints_count', 0),
                    'processing_time': result.get('total_processing_time', 0),
                    'reference_used': result.get('reference_name', 'reference'),
                    'bidirectional_enabled': result.get('bidirectional_enabled', bidirectional),
                    'device_used': result.get('device_used', 'unknown'),
                    'bidirectional_stats': result.get('bidirectional_stats')
                }
                
                return tracking_data
            else:
                logger.error(f"Tracking failed: {result.get('error', 'Unknown error')}")
                return None
                
        except Exception as e:
            logger.error(f"Error tracking keypoints in {target_image_path}: {str(e)}")
            logger.error(traceback.format_exc())
            return None
    
    def save_tracking_result(self, image_name: str, tracking_data: Dict) -> bool:
        """Save tracking result to JSON file.
        
        Args:
            image_name: Name of the target image (without extension)
            tracking_data: Tracking result data
            
        Returns:
            bool: True if saved successfully, False otherwise
        """
        try:
            output_path = os.path.join(self.positioning_data_path, f"{image_name}_tracking_result.json")
            
            # Format the result
            result = {
                "image_file": f"{image_name}.jpg",
                "tracking_result": {
                    "success": True,
                    "tracked_keypoints": tracking_data.get('tracked_keypoints', []),
                    "keypoints_count": tracking_data.get('keypoints_count', 0),
                    "processing_time": tracking_data.get('processing_time', 0),
                    "reference_used": tracking_data.get('reference_used'),
                    "bidirectional_enabled": tracking_data.get('bidirectional_enabled', False),
                    "device_used": tracking_data.get('device_used', 'unknown'),
                    "timestamp": time.strftime('%Y-%m-%d %H:%M:%S')
                }
            }
            
            # Add bidirectional stats if available
            if tracking_data.get('bidirectional_stats'):
                result["tracking_result"]["bidirectional_stats"] = tracking_data['bidirectional_stats']
            
            with open(output_path, 'w') as f:
                json.dump(result, f, indent=2, ensure_ascii=False)
            
            logger.info(f"💾 Saved tracking result to: {output_path}")
            return True
            
        except Exception as e:
            logger.error(f"Failed to save tracking result for {image_name}: {str(e)}")
            return False
    
    def draw_keypoints_on_image(self, image_path: str, tracking_data: Dict, output_name: str) -> bool:
        """Draw tracked keypoints on image and save visualization result.
        
        Args:
            image_path: Path to the original image
            tracking_data: Tracking result data containing keypoints
            output_name: Name for the output image (without extension)
            
        Returns:
            bool: True if visualization saved successfully, False otherwise
        """
        try:
            # Load the image
            if not os.path.exists(image_path):
                logger.error(f"Image file not found: {image_path}")
                return False
            
            image = cv2.imread(image_path)
            if image is None:
                logger.error(f"Failed to load image: {image_path}")
                return False
            
            # Get tracked keypoints
            tracked_keypoints = tracking_data.get('tracked_keypoints', [])
            if not tracked_keypoints:
                logger.warning("No tracked keypoints found for visualization")
                return False
            
            # Define colors for different keypoints (BGR format for OpenCV)
            colors = [
                (0, 255, 0),    # Green
                (255, 0, 0),    # Blue
                (0, 0, 255),    # Red
                (255, 255, 0),  # Cyan
                (255, 0, 255),  # Magenta
                (0, 255, 255),  # Yellow
                (128, 0, 128),  # Purple
                (255, 165, 0),  # Orange
                (0, 128, 255),  # Light Blue
                (128, 255, 0),  # Light Green
            ]
            
            # Draw keypoints
            for i, kp in enumerate(tracked_keypoints):
                x = int(float(kp.get('x', 0)))
                y = int(float(kp.get('y', 0)))
                label = kp.get('label', f"Point {i+1}")
                confidence = kp.get('confidence', 1.0)
                
                # Choose color (cycle through available colors)
                color = colors[i % len(colors)]
                
                # Draw circle for keypoint
                cv2.circle(image, (x, y), 8, color, -1)  # Filled circle
                cv2.circle(image, (x, y), 10, (255, 255, 255), 2)  # White border
                
                # Add confidence score if available (small text near the point)
                if confidence < 1.0:
                    conf_text = f"{confidence:.2f}"
                    cv2.putText(image, conf_text, (x + 12, y + 5), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)
            
            # Create color legend
            legend_start_y = 80
            legend_x = 20
            legend_width = 200
            # Calculate legend height based on number of keypoints
            legend_height = len(tracked_keypoints) * 25 + 40
            
            # Adjust legend position if it would go beyond image boundaries
            if legend_start_y + legend_height > image.shape[0] - 20:
                legend_start_y = max(80, image.shape[0] - legend_height - 20)
            
            # Draw legend background
            cv2.rectangle(image, 
                         (legend_x - 10, legend_start_y - 10), 
                         (legend_x + legend_width, legend_start_y + legend_height), 
                         (0, 0, 0), -1)  # Black background
            cv2.rectangle(image, 
                         (legend_x - 10, legend_start_y - 10), 
                         (legend_x + legend_width, legend_start_y + legend_height), 
                         (255, 255, 255), 2)  # White border
            
            # Legend title
            cv2.putText(image, "Keypoints:", (legend_x, legend_start_y + 15), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            
            # Draw legend items
            for i, kp in enumerate(tracked_keypoints):
                label = kp.get('label', f"Point {i+1}")
                color = colors[i % len(colors)]
                
                # Draw color circle
                cv2.circle(image, (legend_x + 8, legend_start_y + 35 + i * 25), 6, color, -1)
                cv2.circle(image, (legend_x + 8, legend_start_y + 35 + i * 25), 8, (255, 255, 255), 1)
                
                # Draw label text
                cv2.putText(image, label, (legend_x + 25, legend_start_y + 40 + i * 25), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            
            # Add summary information
            summary_text = f"Tracked: {len(tracked_keypoints)} keypoints"
            processing_time = tracking_data.get('processing_time', 0)
            time_text = f"Time: {processing_time:.3f}s"
            
            # Draw summary background
            cv2.rectangle(image, (10, 10), (300, 60), (0, 0, 0), -1)
            cv2.putText(image, summary_text, (20, 35), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.putText(image, time_text, (20, 55), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            
            # Save visualization result
            output_path = os.path.join(self.visualization_dir, f"{output_name}_track_result.jpg")
            success = cv2.imwrite(output_path, image)
            
            if success:
                logger.info(f"🎨 Saved keypoints visualization: {output_path}")
                return True
            else:
                logger.error(f"Failed to save visualization: {output_path}")
                return False
                
        except Exception as e:
            logger.error(f"Error creating keypoints visualization: {str(e)}")
            logger.error(traceback.format_exc())
            return False
    
    def process_all_target_images(self, target_image_names: List[str] = None, bidirectional: bool = False) -> Dict[str, bool]:
        """Process all target images for keypoint tracking.
        
        Args:
            target_image_names: List of image names (without extension). If None, uses 1-5
            bidirectional: Whether to enable bidirectional validation
            
        Returns:
            Dict[str, bool]: Dictionary mapping image names to success status
        """
        if target_image_names is None:
            target_image_names = ['1', '2', '3', '4', '5']
        
        results = {}
        
        logger.info(f"🚀 Starting keypoint tracking for {len(target_image_names)} images")
        logger.info(f"Bidirectional validation: {'Enabled' if bidirectional else 'Disabled'}")
        
        for image_name in target_image_names:
            logger.info(f"\n📸 Processing image: {image_name}.jpg")
            
            target_path = os.path.join(self.positioning_data_path, f"{image_name}.jpg")
            
            # Track keypoints
            tracking_data = self.track_keypoints_in_image(target_path, bidirectional)
            
            if tracking_data:
                # Save result
                save_success = self.save_tracking_result(image_name, tracking_data)
                
                # Create visualization
                viz_success = self.draw_keypoints_on_image(target_path, tracking_data, image_name)
                
                # Consider success if both tracking result and visualization are saved
                results[image_name] = save_success and viz_success
                
                if viz_success:
                    logger.info(f"✅ Created visualization for {image_name}.jpg")
                else:
                    logger.warning(f"⚠️ Failed to create visualization for {image_name}.jpg")
            else:
                results[image_name] = False
                logger.error(f"❌ Failed to track keypoints in {image_name}.jpg")
        
        return results
    
    def run_tracking_pipeline(self, bidirectional: bool = False) -> bool:
        """Run the complete keypoint tracking pipeline.
        
        Args:
            bidirectional: Whether to enable bidirectional validation
            
        Returns:
            bool: True if pipeline completed successfully, False otherwise
        """
        try:
            logger.info("🎯 Starting Keypoint Tracking Pipeline")
            logger.info("="*50)
            
            # Step 1: Check API health
            logger.info("Step 1: Checking API health...")
            if not self.check_api_health():
                logger.error("❌ API health check failed. Make sure the robot_vision API is running.")
                return False
            
            # Step 2: Load reference keypoints
            logger.info("\nStep 2: Loading reference keypoints...")
            keypoints = self.load_reference_keypoints()
            
            # Step 3: Set reference image
            logger.info("\nStep 3: Setting reference image...")
            if not self.set_reference_image(keypoints):
                logger.error("❌ Failed to set reference image")
                return False
            
            # Step 4: Process target images
            logger.info("\nStep 4: Processing target images...")
            results = self.process_all_target_images(bidirectional=bidirectional)
            
            # Step 5: Summary
            logger.info("\n" + "="*50)
            logger.info("📊 TRACKING SUMMARY")
            logger.info("="*50)
            
            successful = sum(1 for success in results.values() if success)
            total = len(results)
            
            logger.info(f"Total images processed: {total}")
            logger.info(f"Successful (with visualization): {successful}")
            logger.info(f"Failed: {total - successful}")
            
            for image_name, success in results.items():
                status = "✅ Success" if success else "❌ Failed"
                logger.info(f"  {image_name}.jpg: {status}")
            
            if successful > 0:
                logger.info(f"\n🎨 Visualization results saved to: {self.visualization_dir}")
                logger.info("Files created:")
                for image_name, success in results.items():
                    if success:
                        logger.info(f"  - {image_name}_track_result.jpg")
            
            pipeline_success = successful == total
            if pipeline_success:
                logger.info("🎉 Keypoint tracking pipeline completed successfully!")
            else:
                logger.warning("⚠️ Keypoint tracking pipeline completed with some failures.")
            
            return pipeline_success
            
        except Exception as e:
            logger.error(f"Pipeline failed with error: {str(e)}")
            logger.error(traceback.format_exc())
            return False


def main():
    """Main function to run the keypoint tracking task."""
    logger.info("🎯 Keypoint Tracking Task - Direct FFPPWebAPIKeypointTracker")
    logger.info("="*60)
    
    # Get service URL from environment variable or command line argument
    service_url = "http://msraig-ubuntu-3:8001"  # default FlowFormer++ service URL
    
    if len(sys.argv) > 1:
        service_url = sys.argv[1]
        logger.info(f"Using service URL from command line: {service_url}")
    elif "FLOWFORMER_SERVICE_URL" in os.environ:
        service_url = os.environ["FLOWFORMER_SERVICE_URL"]
        logger.info(f"Using service URL from environment: {service_url}")
    else:
        logger.info(f"Using default FlowFormer++ service URL: {service_url}")
        logger.info("You can specify a different URL with:")
        logger.info("  python3 estimation_task_keypoints_track.py http://your-server:8001")
        logger.info("  or set FLOWFORMER_SERVICE_URL environment variable")
    
    # Initialize tracker
    try:
        tracker = KeypointTrackerInTaskEstimation(service_url=service_url)
    except ImportError as e:
        logger.error(f"Failed to initialize tracker: {e}")
        logger.error("Make sure you have the robot_vision module available")
        sys.exit(1)
    except Exception as e:
        logger.error(f"Failed to initialize tracker: {e}")
        sys.exit(1)
    
    # Run the tracking pipeline
    success = tracker.run_tracking_pipeline(bidirectional=False)
    
    if success:
        logger.info("✅ Task completed successfully!")
        sys.exit(0)
    else:
        logger.error("❌ Task failed!")
        sys.exit(1)


if __name__ == "__main__":
    main()
