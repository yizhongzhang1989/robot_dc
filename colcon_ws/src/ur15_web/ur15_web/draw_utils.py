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
        z_threshold = 1e-6
        in_front = points_cam[:, 2] > z_threshold
        
        # Project all 3D points onto the image plane
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
        
        # Draw lines connecting consecutive points, but only if both endpoints are in front
        for i in range(len(projected_points) - 1):
            # Only draw if both points are in front of the camera
            if in_front[i] and in_front[i + 1]:
                pt1 = tuple(projected_points[i])
                pt2 = tuple(projected_points[i + 1])
                cv2.line(image, pt1, pt2, curve_color, curve_thickness)
    
    return image

