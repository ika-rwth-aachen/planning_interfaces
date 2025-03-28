from geometry_msgs.msg import TransformStamped, PointStamped
from route_planning_msgs.msg import Route, RouteElement, LaneElement, RegulatoryElement, LaneBoundary

import tf2_ros
import tf2_geometry_msgs

def to_msg_msg(msg):
    return msg

tf2_ros.ConvertRegistration().add_to_msg(Route, to_msg_msg)
tf2_ros.ConvertRegistration().add_to_msg(RouteElement, to_msg_msg)
tf2_ros.ConvertRegistration().add_to_msg(LaneElement, to_msg_msg)
tf2_ros.ConvertRegistration().add_to_msg(RegulatoryElement, to_msg_msg)
tf2_ros.ConvertRegistration().add_to_msg(LaneBoundary, to_msg_msg)

def from_msg_msg(msg):
    return msg

tf2_ros.ConvertRegistration().add_from_msg(Route, from_msg_msg)
tf2_ros.ConvertRegistration().add_from_msg(RouteElement, from_msg_msg)
tf2_ros.ConvertRegistration().add_from_msg(LaneElement, from_msg_msg)
tf2_ros.ConvertRegistration().add_from_msg(RegulatoryElement, from_msg_msg)
tf2_ros.ConvertRegistration().add_from_msg(LaneBoundary, from_msg_msg)

def do_transform_lane_boundary(msg: LaneBoundary, transform: TransformStamped) -> LaneBoundary:
    """
    Apply a `Transform` or `TransformStamped` to `LaneBoundary`.
    :param msg: The msg that should be transformed
    :param transform: The transform which will applied to the msg
    :returns: The transformed msg
    """
    msg_out = LaneBoundary()
    msg_out = msg

    # Transform the point
    point = PointStamped()
    point.point = msg.point
    msg_out.point = tf2_geometry_msgs.do_transform_point(point, transform).point

    return msg_out

tf2_ros.TransformRegistration().add(LaneBoundary, do_transform_lane_boundary)

def do_transform_lane_element(msg: LaneElement, transform: TransformStamped) -> LaneElement:
    """
    Apply a `Transform` or `TransformStamped` to `LaneElement`.
    :param msg: The msg that should be transformed
    :param transform: The transform which will applied to the msg
    :returns: The transformed msg
    """
    msg_out = LaneElement()
    msg_out = msg

    # reference_pose
    msg_out.reference_pose = tf2_geometry_msgs.do_transform_pose(msg.reference_pose, transform)

    # lane_boundaries
    if msg.has_left_boundary:
        msg_out.left_boundary = do_transform_lane_boundary(msg.left_boundary, transform)
    if msg.has_right_boundary:
        msg_out.right_boundary = do_transform_lane_boundary(msg.right_boundary, transform)

    return msg_out

tf2_ros.TransformRegistration().add(LaneElement, do_transform_lane_element)

def do_transform_regulatory_element(msg: RegulatoryElement, transform: TransformStamped) -> RegulatoryElement:
    """
    Apply a `Transform` or `TransformStamped` to `RegulatoryElement`.
    :param msg: The msg that should be transformed
    :param transform: The transform which will applied to the msg
    :returns: The transformed msg
    """
    msg_out = RegulatoryElement()
    msg_out = msg

    # effect_line
    for i in range(len(msg.effect_line)):
        point = PointStamped()
        point.point = msg.effect_line[i]
        msg_out.effect_line[i] = tf2_geometry_msgs.do_transform_point(point, transform).point

    # sign_positions
    for i in range(len(msg.sign_positions)):
        point = PointStamped()
        point.point = msg.sign_positions[i]
        msg_out.sign_positions[i] = tf2_geometry_msgs.do_transform_point(point, transform).point

    return msg_out

tf2_ros.TransformRegistration().add(RegulatoryElement, do_transform_regulatory_element)

def do_transform_route_element(msg: RouteElement, transform: TransformStamped) -> RouteElement:
    """
    Apply a `Transform` or `TransformStamped` to `RouteElement`.
    :param msg: The msg that should be transformed
    :param transform: The transform which will applied to the msg
    :returns: The transformed msg
    """
    msg_out = RouteElement()
    msg_out = msg

    # lane_elements
    for i in range(len(msg.lane_elements)):
        msg_out.lane_elements[i] = do_transform_lane_element(msg.lane_elements[i], transform)

    # boundaries (drivable space)
    point = PointStamped()
    point.point = msg.left_boundary
    msg_out.left_boundary = tf2_geometry_msgs.do_transform_point(point, transform).point
    point.point = msg.right_boundary
    msg_out.right_boundary = tf2_geometry_msgs.do_transform_point(point, transform).point

    # regulatory_elements
    for i in range(len(msg.regulatory_elements)):
        msg_out.regulatory_elements[i] = do_transform_regulatory_element(msg.regulatory_elements[i], transform)

    return msg_out

tf2_ros.TransformRegistration().add(RouteElement, do_transform_route_element)

def do_transform_route(msg: Route, transform: TransformStamped) -> Route:
    """
    Apply a `Transform` or `TransformStamped` to `Route`.
    :param msg: The msg that should be transformed
    :param transform: The transform which will applied to the msg
    :returns: The transformed msg
    """
    msg_out = Route()
    msg_out = msg
    msg_out.header.stamp = transform.header.stamp
    msg_out.header.frame_id = transform.header.frame_id

    # destination
    point = PointStamped()
    point.point = msg.destination
    msg_out.destination = tf2_geometry_msgs.do_transform_point(point, transform).point

    # traveled route_elements
    for i in range(len(msg.traveled_route_elements)):
        msg_out.traveled_route_elements[i] = do_transform_route_element(msg.traveled_route_elements[i], transform)

    # remaining route_elements
    for i in range(len(msg.remaining_route_elements)):
        msg_out.remaining_route_elements[i] = do_transform_route_element(msg.remaining_route_elements[i], transform)

    return msg_out

tf2_ros.TransformRegistration().add(Route, do_transform_route)