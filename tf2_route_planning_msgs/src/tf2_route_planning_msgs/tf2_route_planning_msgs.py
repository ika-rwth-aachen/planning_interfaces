from copy import deepcopy
from typing import Iterable

from geometry_msgs.msg import Point, PointStamped, TransformStamped
from route_planning_msgs.msg import (
    LaneBoundary,
    LaneElement,
    RegulatoryElement,
    Route,
    RouteElement,
)

import tf2_geometry_msgs
import tf2_ros


def _register_passthrough_conversions(message_types: Iterable[type]) -> None:
    def _to_msg(msg):
        return msg

    def _from_msg(msg):
        return msg

    for msg_type in message_types:
        tf2_ros.ConvertRegistration().add_to_msg(msg_type, _to_msg)
        tf2_ros.ConvertRegistration().add_from_msg(msg_type, _from_msg)


_register_passthrough_conversions(
    (Route, RouteElement, LaneElement, RegulatoryElement, LaneBoundary)
)


def _transform_point(point: Point, transform: TransformStamped) -> Point:
    stamped = PointStamped()
    stamped.point = deepcopy(point)
    transformed = tf2_geometry_msgs.do_transform_point(stamped, transform)
    return transformed.point



def do_transform_lane_boundary(msg: LaneBoundary, transform: TransformStamped) -> LaneBoundary:
    """
    Apply a `Transform` or `TransformStamped` to `LaneBoundary`.
    :param msg: The msg that should be transformed
    :param transform: The transform which will applied to the msg
    :returns: The transformed msg
    """
    msg_out = deepcopy(msg)
    msg_out.point = _transform_point(msg.point, transform)
    return msg_out

tf2_ros.TransformRegistration().add(LaneBoundary, do_transform_lane_boundary)

def do_transform_lane_element(msg: LaneElement, transform: TransformStamped) -> LaneElement:
    """
    Apply a `Transform` or `TransformStamped` to `LaneElement`.
    :param msg: The msg that should be transformed
    :param transform: The transform which will applied to the msg
    :returns: The transformed msg
    """
    msg_out = deepcopy(msg)

    # reference_pose
    msg_out.reference_pose = tf2_geometry_msgs.do_transform_pose(msg.reference_pose, transform)

    # lane_boundaries
    msg_out.left_boundary = do_transform_lane_boundary(msg.left_boundary, transform)
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
    msg_out = deepcopy(msg)

    # reference_line
    for i in range(len(msg.reference_line)):
        msg_out.reference_line[i] = _transform_point(msg.reference_line[i], transform)

    # positions
    for i in range(len(msg.positions)):
        msg_out.positions[i] = _transform_point(msg.positions[i], transform)

    return msg_out

tf2_ros.TransformRegistration().add(RegulatoryElement, do_transform_regulatory_element)

def do_transform_route_element(msg: RouteElement, transform: TransformStamped) -> RouteElement:
    """
    Apply a `Transform` or `TransformStamped` to `RouteElement`.
    :param msg: The msg that should be transformed
    :param transform: The transform which will applied to the msg
    :returns: The transformed msg
    """
    msg_out = deepcopy(msg)

    # lane_elements
    for i in range(len(msg.lane_elements)):
        msg_out.lane_elements[i] = do_transform_lane_element(msg.lane_elements[i], transform)

    # boundaries (drivable space)
    msg_out.left_boundary = _transform_point(msg.left_boundary, transform)
    msg_out.right_boundary = _transform_point(msg.right_boundary, transform)

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
    msg_out = deepcopy(msg)
    msg_out.header.stamp = transform.header.stamp
    msg_out.header.frame_id = transform.header.frame_id

    # destination
    msg_out.destination = _transform_point(msg.destination, transform)

    # intermediate destinations
    for i in range(len(msg.intermediate_destinations)):
        msg_out.intermediate_destinations[i] = _transform_point(msg.intermediate_destinations[i], transform)

    # route_elements
    for i in range(len(msg.route_elements)):
        msg_out.route_elements[i] = do_transform_route_element(msg.route_elements[i], transform)
 
    return msg_out

tf2_ros.TransformRegistration().add(Route, do_transform_route)
