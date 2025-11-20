# ============================================================================
# MIT License
# 
# Copyright (c) 2025 Institute for Automotive Engineering (ika), RWTH Aachen University
# 
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
# 
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
# 
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
# ============================================================================

from route_planning_msgs.msg import LaneBoundary, LaneElement
from geometry_msgs.msg import Point


def set_lane_boundary(boundary: LaneBoundary, point: Point, type: int = LaneBoundary.TYPE_UNKNOWN) -> None:
    """
    Set the position of a LaneBoundary to the given point.

    :param boundary: The LaneBoundary to set.
    :param point: The Point to set the LaneBoundary to.
    :param type: The type of the LaneBoundary. Defaults to TYPE_UNKNOWN.
    """
    boundary.point = point
    if type > 4:
        raise ValueError(f"Invalid lane boundary type: {type}")
    boundary.type = type

def set_left_boundary_of_lane_element(lane_element: LaneElement, lane_boundary: LaneBoundary):
    """
    Set the left boundary of a LaneElement to the given LaneBoundary.

    :param lane_element: The LaneElement to set the left boundary for.
    :param lane_boundary: The LaneBoundary to set as the left boundary.
    :return: The updated LaneElement.
    """
    lane_element.left_boundary = lane_boundary

def set_left_boundary_of_lane_element_from_point(lane_element: LaneElement, point: Point, type: int=LaneBoundary.TYPE_UNKNOWN):
    """
    Set the left boundary of a LaneElement to the given point.

    :param lane_element: The LaneElement to set the left boundary for.
    :param point: The Point to set the left boundary to.
    :param type: The type of the LaneBoundary. Default is TYPE_UNKNOWN.
    :return: The updated LaneElement.
    """
    boundary = LaneBoundary()
    set_lane_boundary(boundary, point, type)
    set_left_boundary_of_lane_element(lane_element, boundary)

def set_right_boundary_of_lane_element(lane_element: LaneElement, lane_boundary: LaneBoundary):
    """
    Set the right boundary of a LaneElement to the given LaneBoundary.

    :param lane_element: The LaneElement to set the right boundary for.
    :param lane_boundary: The LaneBoundary to set as the right boundary.
    :return: The updated LaneElement.
    """
    lane_element.right_boundary = lane_boundary

def set_right_boundary_of_lane_element_from_point(lane_element: LaneElement, point: Point, type: int=LaneBoundary.TYPE_UNKNOWN):
    """
    Set the right boundary of a LaneElement to the given point.

    :param lane_element: The LaneElement to set the right boundary for.
    :param point: The Point to set the right boundary to.
    :param type: The type of the LaneBoundary. Default is TYPE_UNKNOWN.
    :return: The updated LaneElement.
    """
    boundary = LaneBoundary()
    set_lane_boundary(boundary, point, type)
    set_right_boundary_of_lane_element(lane_element, boundary)
