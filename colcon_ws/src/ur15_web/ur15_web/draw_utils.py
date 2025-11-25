import numpy as np
import cv2


def draw_curves_on_image(image, intrinsic, extrinsic, point3d, distortion=None, color=(0, 0, 255), thickness=1):
    """
    Draw one or multiple connected curves on an image by projecting 3D points onto the image plane.
    
    This function robustly handles cases where 3D points are behind the camera by:
    - Transforming points to camera coordinate system
    - Checking if points are in front of camera (Z > 0)
    - Only drawing line segments where both endpoints are visible
    
    Args:
        image: Input image (numpy array) to draw on
        intrinsic: Camera intrinsic matrix (3x3)
        extrinsic: Extrinsic matrix (4x4) containing [R|t]
        point3d: Single curve (array/list of points) or list of curves.
                 Each curve should be Nx3 array or list of [x,y,z] points.
        distortion: Distortion coefficients (1D array) or None (default: None)
        color: Single BGR color tuple or list of colors (default: (0, 0, 255) red).
               If list is shorter than curves, uses last color for remaining curves.
        thickness: Single int or list of ints for line thickness (default: 1).
                   If list is shorter than curves, uses last thickness for remaining curves.
        
    Returns:
        image: The image with the curve(s) drawn on it
    """
    # Determine if point3d is a single curve or multiple curves
    # Check if it's a list of curves by examining the structure
    is_multiple_curves = False
    if isinstance(point3d, list) and len(point3d) > 0:
        # Check if first element is itself a list/array (indicating multiple curves)
        first_elem = point3d[0]
        if isinstance(first_elem, (list, np.ndarray)):
            # Further check: if first element has shape info or is a list
            if isinstance(first_elem, np.ndarray):
                # If it's a 2D array or 1D array with length > 3, likely multiple curves
                is_multiple_curves = len(first_elem) > 3 or (hasattr(first_elem, 'ndim') and first_elem.ndim >= 1 and len(first_elem) != 3)
            elif isinstance(first_elem, list):
                # If first element is a list, check if it contains numbers or lists
                if len(first_elem) > 0 and isinstance(first_elem[0], (list, np.ndarray)):
                    is_multiple_curves = True
                elif len(first_elem) != 3:
                    # If the inner list is not length 3, likely multiple curves
                    is_multiple_curves = True
    
    # If it's a numpy array, check dimensions
    if isinstance(point3d, np.ndarray) and point3d.ndim == 3:
        is_multiple_curves = True
    
    # Convert to list of curves format
    if is_multiple_curves:
        curves = point3d
    else:
        curves = [point3d]
    
    # Normalize color to list
    if isinstance(color, tuple) or (isinstance(color, np.ndarray) and color.ndim == 1):
        colors = [color]
    else:
        colors = color
    
    # Normalize thickness to list
    if isinstance(thickness, int):
        thicknesses = [thickness]
    else:
        thicknesses = thickness
    
    # Draw each curve
    for idx, curve_points in enumerate(curves):
        # Get color and thickness for this curve (use last if index out of range)
        curve_color = colors[min(idx, len(colors) - 1)]
        curve_thickness = thicknesses[min(idx, len(thicknesses) - 1)]
        
        # Convert curve points to numpy array
        points_3d = np.asarray(curve_points, dtype=np.float32).reshape(-1, 3)
        
        # Check if we have at least 2 points (after reshaping)
        if len(points_3d) < 2:
            continue
        
        # Extract rotation and translation from extrinsic matrix
        rvec, _ = cv2.Rodrigues(extrinsic[:3, :3])
        tvec = extrinsic[:3, 3].reshape(3, 1)
        
        # Transform 3D points to camera coordinate system to check Z-depth
        rotation_matrix = extrinsic[:3, :3]
        translation_vector = extrinsic[:3, 3]
        points_cam = (rotation_matrix @ points_3d.T).T + translation_vector
        
        # Check which points are in front of camera (positive Z in camera frame)
        # Use a small epsilon to avoid numerical issues at exactly Z=0
        z_threshold = 1e-2
        in_front = points_cam[:, 2] > z_threshold
        
        # Project points without distortion to check if they're within image bounds
        # This filters points outside the visual cone before distortion is applied
        projected_no_distortion, _ = cv2.projectPoints(
            points_3d,
            rvec,
            tvec,
            intrinsic,
            np.zeros(5, dtype=np.float32)  # No distortion
        )
        projected_no_distortion = projected_no_distortion.reshape(-1, 2)
        
        # Get image dimensions
        img_height, img_width = image.shape[:2]
        
        # Allow points outside image by 1/4 of image dimensions
        margin_x = img_width * 0.1
        margin_y = img_height * 0.1
        
        # Check which points project within expanded image bounds (without distortion)
        within_image = (
            (projected_no_distortion[:, 0] >= -margin_x) &
            (projected_no_distortion[:, 0] < img_width + margin_x) &
            (projected_no_distortion[:, 1] >= -margin_y) &
            (projected_no_distortion[:, 1] < img_height + margin_y)
        )
        
        # Combine visibility checks: in front of camera AND within image bounds
        visible = in_front & within_image
        
        # Project all 3D points onto the image plane with distortion
        # Handle None distortion by using zero distortion coefficients
        dist_coeffs = distortion if distortion is not None else np.zeros(5, dtype=np.float32)
        projected_points, _ = cv2.projectPoints(
            points_3d,
            rvec,
            tvec,
            intrinsic,
            dist_coeffs
        )
        
        # Convert to integer pixel coordinates
        projected_points = projected_points.reshape(-1, 2).astype(int)
        
        # Draw lines connecting consecutive points, but only if both endpoints are visible
        for i in range(len(projected_points) - 1):
            # Only draw if both points are visible (in front AND within image bounds)
            if visible[i] and visible[i + 1]:
                pt1 = tuple(projected_points[i])
                pt2 = tuple(projected_points[i + 1])
                cv2.line(image, pt1, pt2, curve_color, curve_thickness)
    
    return image


def generate_ur15_base_curve(num_points=73, num_rays=16, ray_length=1.0):
    """
    Generate the shape representing the base circle and plane of UR15 robot.
    
    Args:
        num_points: Number of points for the circle (default: 73)
        num_rays: Number of rays emanating from the circle (default: 16)
        ray_length: Length of each ray extending from the circle (default: 0.05)
    
    Returns:
        dict: Dictionary with keys 'curves' and 'colors'
              - curves: List of curve point arrays
              - colors: List of corresponding BGR colors
    """
    radius = 0.102  # Base circle radius for UR15
    curves = []
    colors = []
    
    # Generate the base circle on x-y plane at z=0
    circle_points = []
    for i in range(num_points):
        angle = 2 * np.pi * i / (num_points - 1)  # Last point overlaps first
        x = radius * np.cos(angle)
        y = radius * np.sin(angle)
        z = 0.0
        circle_points.append([x, y, z])
    
    curves.append(circle_points)
    colors.append((0, 0, 255))  # Red for circle
    
    # Generate rays emanating from the circle
    # Each ray must be divided into segments no longer than 0.1
    max_segment_length = 0.1
    
    for i in range(num_rays):
        angle = 2 * np.pi * i / num_rays
        
        # Starting point on the circle
        x_start = radius * np.cos(angle)
        y_start = radius * np.sin(angle)
        z_start = 0.0
        
        # Ending point outside the circle (ray extends radially)
        x_end = (radius + ray_length) * np.cos(angle)
        y_end = (radius + ray_length) * np.sin(angle)
        z_end = 0.0
        
        # Calculate number of segments needed
        num_segments = int(np.ceil(ray_length / max_segment_length))
        if num_segments < 1:
            num_segments = 1
        
        # Generate ray points with multiple segments
        ray_points = []
        for j in range(num_segments + 1):
            t = j / num_segments  # Parameter from 0 to 1
            x = x_start + t * (x_end - x_start)
            y = y_start + t * (y_end - y_start)
            z = z_start + t * (z_end - z_start)
            ray_points.append([x, y, z])
        
        curves.append(ray_points)
        colors.append((0, 255, 0))  # Green for rays
    
    return {
        'curves': curves,
        'colors': colors
    }


def generate_gb200rack_curve():
    """
    Generate the shape representing a GB200 rack as a square on the x-z plane.
    Lower left corner at (0, 0, 0), upper right corner at (0.55, 0, 2.145).
    
    Returns:
        dict: Dictionary with keys 'curves' and 'colors'
              - curves: List of curve point arrays (one curve forming the rectangle)
              - colors: List of corresponding BGR colors
    """
    # Define the corners of the rectangle on x-z plane (y=0)
    x_min, x_max = 0.0, 0.55
    z_min, z_max = 0.0, 2.145
    
    max_segment_length = 0.1
    
    # Calculate number of segments needed for each side
    width = x_max - x_min
    height = z_max - z_min
    
    num_segments_horizontal = int(np.ceil(width / max_segment_length))
    num_segments_vertical = int(np.ceil(height / max_segment_length))
    
    if num_segments_horizontal < 1:
        num_segments_horizontal = 1
    if num_segments_vertical < 1:
        num_segments_vertical = 1
    
    square_points = []
    
    # Bottom edge: from (x_min, 0, z_min) to (x_max, 0, z_min)
    for i in range(num_segments_horizontal + 1):
        t = i / num_segments_horizontal
        x = x_min + t * width
        y = 0.0
        z = z_min
        square_points.append([x, y, z])
    
    # Right edge: from (x_max, 0, z_min) to (x_max, 0, z_max)
    # Skip first point to avoid duplication
    for i in range(1, num_segments_vertical + 1):
        t = i / num_segments_vertical
        x = x_max
        y = 0.0
        z = z_min + t * height
        square_points.append([x, y, z])
    
    # Top edge: from (x_max, 0, z_max) to (x_min, 0, z_max)
    # Skip first point to avoid duplication
    for i in range(1, num_segments_horizontal + 1):
        t = i / num_segments_horizontal
        x = x_max - t * width
        y = 0.0
        z = z_max
        square_points.append([x, y, z])
    
    # Left edge: from (x_min, 0, z_max) to (x_min, 0, z_min)
    # Skip first and last points to avoid duplication
    for i in range(1, num_segments_vertical):
        t = i / num_segments_vertical
        x = x_min
        y = 0.0
        z = z_max - t * height
        square_points.append([x, y, z])
    
    # Close the square by adding the first point again
    square_points.append(square_points[0])
    
    return {
        'curves': [square_points],
        'colors': [(0, 0, 255)]  # Red for the square
    }


def draw_keypoints_on_image(image, intrinsic, extrinsic, points_3d, distortion=None, 
                            radius=6, color=(255, 105, 180), thickness=2, 
                            draw_labels=True, label_color=(255, 255, 255)):
    """
    Draw 3D keypoints on an image by projecting them onto the image plane.
    
    This function robustly handles cases where 3D points are behind the camera by:
    - Transforming points to camera coordinate system
    - Checking if points are in front of camera (Z > 0)
    - Only drawing keypoints that are visible
    
    Args:
        image: Input image (numpy array) to draw on
        intrinsic: Camera intrinsic matrix (3x3)
        extrinsic: Extrinsic matrix (4x4) containing [R|t]
        points_3d: Array/list of 3D points, shape (N, 3) or list of [x,y,z]
        distortion: Distortion coefficients (1D array) or None (default: None)
        radius: Radius of the keypoint circle in pixels (default: 6)
        color: BGR color tuple for the keypoint (default: (255, 105, 180) magenta/pink)
        thickness: Thickness for the outer circle (default: 2). Use -1 for filled circle.
        draw_labels: Whether to draw index labels next to keypoints (default: True)
        label_color: BGR color tuple for labels (default: (255, 255, 255) white)
        
    Returns:
        image: The image with the keypoints drawn on it
    """
    # Convert points to numpy array
    points_3d = np.asarray(points_3d, dtype=np.float32).reshape(-1, 3)
    
    # Check if we have any points
    if len(points_3d) == 0:
        return image
    
    # Extract rotation and translation from extrinsic matrix
    rvec, _ = cv2.Rodrigues(extrinsic[:3, :3])
    tvec = extrinsic[:3, 3].reshape(3, 1)
    
    # Transform 3D points to camera coordinate system to check Z-depth
    rotation_matrix = extrinsic[:3, :3]
    translation_vector = extrinsic[:3, 3]
    points_cam = (rotation_matrix @ points_3d.T).T + translation_vector
    
    # Check which points are in front of camera (positive Z in camera frame)
    z_threshold = 1e-2
    in_front = points_cam[:, 2] > z_threshold
    
    # Project points without distortion to check if they're within image bounds
    projected_no_distortion, _ = cv2.projectPoints(
        points_3d,
        rvec,
        tvec,
        intrinsic,
        np.zeros(5, dtype=np.float32)
    )
    projected_no_distortion = projected_no_distortion.reshape(-1, 2)
    
    # Get image dimensions
    img_height, img_width = image.shape[:2]
    
    # Allow points outside image by 10% of image dimensions for margin
    margin_x = img_width * 0.1
    margin_y = img_height * 0.1
    
    # Check which points project within expanded image bounds
    within_image = (
        (projected_no_distortion[:, 0] >= -margin_x) &
        (projected_no_distortion[:, 0] < img_width + margin_x) &
        (projected_no_distortion[:, 1] >= -margin_y) &
        (projected_no_distortion[:, 1] < img_height + margin_y)
    )
    
    # Combine visibility checks: in front of camera AND within image bounds
    visible = in_front & within_image
    
    # Project all 3D points onto the image plane with distortion
    dist_coeffs = distortion if distortion is not None else np.zeros(5, dtype=np.float32)
    projected_points, _ = cv2.projectPoints(
        points_3d,
        rvec,
        tvec,
        intrinsic,
        dist_coeffs
    )
    
    # Convert to integer pixel coordinates
    projected_points = projected_points.reshape(-1, 2).astype(int)
    
    # Draw each visible keypoint
    for idx, (pt, is_visible) in enumerate(zip(projected_points, visible)):
        if not is_visible:
            continue
        
        pt_tuple = tuple(pt)
        
        # Draw filled circle for keypoint
        cv2.circle(image, pt_tuple, radius, color, -1)
        
        # Draw outer circle for better visibility
        if thickness > 0:
            cv2.circle(image, pt_tuple, radius + 2, (255, 255, 255), thickness)
        
        # Draw point index label if requested
        if draw_labels:
            label = f"{idx}"
            label_pos = (pt[0] + radius + 6, pt[1] + 5)
            
            # Draw text background
            (text_w, text_h), baseline = cv2.getTextSize(
                label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2
            )
            cv2.rectangle(
                image,
                (label_pos[0] - 2, label_pos[1] - text_h - 2),
                (label_pos[0] + text_w + 2, label_pos[1] + baseline + 2),
                (0, 0, 0),
                -1
            )
            
            # Draw text
            cv2.putText(
                image,
                label,
                label_pos,
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                label_color,
                2
            )
    
    return image
