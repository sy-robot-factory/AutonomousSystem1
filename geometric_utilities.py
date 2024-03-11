import numpy
import sys

def point_rect_collision(rect_x, rect_y, rect_w, rect_h, point_x, point_y):
    """
    Check if a point collides with a rectangle defined by rect_x, rect_y, width rect_w, and height rect_h.
    
    Parameters:
    - rect_x: x-coordinate of the rectangle
    - rect_y: y-coordinate of the rectangle
    - rect_w: width of the rectangle
    - rect_h: height of the rectangle
    - point_x: x-coordinate of the point
    - point_y: y-coordinate of the point
    
    Returns:
    - True if the point collides with the rectangle, False otherwise
    """
    
    if point_x >= rect_x and point_x <= rect_x + rect_w and point_y >= rect_y and point_y <= rect_y + rect_h:
        return True
    return False

def rect_rect_collision(rect1_x, rect1_y, rect1_w, rect1_h, rect2_x, rect2_y, rect2_w, rect2_h):
    """
    Check for collision between two rectangles based on their coordinates and dimensions.
    Parameters:
    - rect1_x: x-coordinate of the first rectangle
    - rect1_y: y-coordinate of the first rectangle
    - rect1_w: width of the first rectangle
    - rect1_h: height of the first rectangle
    - rect2_x: x-coordinate of the second rectangle
    - rect2_y: y-coordinate of the second rectangle
    - rect2_w: width of the second rectangle
    - rect2_h: height of the second rectangle
    Returns:
    - True if there is a collision, False otherwise
    """
    if rect1_x + rect1_w >= rect2_x and rect1_x <= rect2_x + rect2_w and rect1_y + rect1_h >= rect2_y and rect1_y <= rect2_y + rect2_h:
        return True
    return False

def circle_rect_collision(circle_x, circle_y, circle_r, rect_x, rect_y, rect_w, rect_h):
    """
    Function to check for collision between a circle and a rectangle.

    Args:
        circle_x (float): x-coordinate of the center of the circle
        circle_y (float): y-coordinate of the center of the circle
        circle_r (float): radius of the circle
        rect_x (float): x-coordinate of the top-left corner of the rectangle
        rect_y (float): y-coordinate of the top-left corner of the rectangle
        rect_w (float): width of the rectangle
        rect_h (float): height of the rectangle

    Returns:
        bool: True if the circle collides with the rectangle, False otherwise
    """
    test_x = circle_x
    test_y = circle_y

    ##find nearest edge
    if circle_x < rect_x: #left
        test_x = rect_x
    elif circle_x > rect_x + rect_w: #right
        test_x = rect_x + rect_w

    if circle_y < rect_y:#bottom
        test_y = rect_y
    elif circle_y > rect_y + rect_h:#top
        test_y = rect_y + rect_h

    c_pos = numpy.array([circle_x, circle_y])
    test_val = numpy.array([test_x, test_y])

    dist = numpy.linalg.norm(c_pos - test_val)

    if dist <= circle_r:
        return True
    
    return False

def get_angle(vector1, vector2):
    """
    Calculate the angle between two vectors.

    Args:
    vector1: The first vector.
    vector2: The second vector.

    Returns:
    The angle between vector1 and vector2 in radians.
    """
    
    unit_v1 = vector1 / numpy.linalg.norm(vector1)
    unit_v2 = vector2 / numpy.linalg.norm(vector2)

    cos_theta = numpy.dot(unit_v1, unit_v2) / (numpy.linalg.norm(unit_v1) * numpy.linalg.norm(unit_v2))
    #cos_theta should be [-1,1]
    cos_theta = numpy.clip(cos_theta, -1.0, 1.0)
    
    angle_rad = numpy.arccos(cos_theta)

    return angle_rad

def get_ray_rect_intersection_distance(ray_origin_x, ray_origin_y, ray_dir_x, ray_dir_y, rect_min_x, rect_min_y, rect_max_x, rect_max_y):
    """
    Calculate the intersection distance between a ray and a rectangle.

    Args:
        ray_origin_x (float): The x-coordinate of the ray's origin.
        ray_origin_y (float): The y-coordinate of the ray's origin.
        ray_dir_x (float): The x-component of the ray's direction.
        ray_dir_y (float): The y-component of the ray's direction.
        rect_min_x (float): The minimum x-coordinate of the rectangle.
        rect_min_y (float): The minimum y-coordinate of the rectangle.
        rect_max_x (float): The maximum x-coordinate of the rectangle.
        rect_max_y (float): The maximum y-coordinate of the rectangle.

    Returns:
        float: The intersection distance between the ray and the rectangle, or -1 if there is no intersection.
    """
    
    inverted_ray_dir_x = 1.0 / ray_dir_x
    inverted_ray_dir_y = 1.0 / ray_dir_y

    t_min = 0.0
    t_max = sys.float_info.max
    
    ##x
    t1 = (rect_min_x - ray_origin_x) * inverted_ray_dir_x
    t2 = (rect_max_x - ray_origin_x) * inverted_ray_dir_x
    t_min = max(t_min, min(t1, t2))
    t_max = min(t_max, max(t1, t2))
    ##y
    t1 = (rect_min_y - ray_origin_y) * inverted_ray_dir_y
    t2 = (rect_max_y - ray_origin_y) * inverted_ray_dir_y
    t_min = max(t_min, min(t1, t2))
    t_max = min(t_max, max(t1, t2))

    ret_val = -1
    if t_max > max(t_min, 0.0):
        ret_val = t_min

    return ret_val



