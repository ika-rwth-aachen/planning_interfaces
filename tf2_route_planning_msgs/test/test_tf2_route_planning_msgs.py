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

import pytest
from geometry_msgs.msg import TransformStamped
from route_planning_msgs.msg import Route
from tf2_route_planning_msgs import do_transform_route
import math

EPS = 1e-12

def test_do_transform_route():
    # Create a Route object
    route = Route()
    route.destination.x = 1.0
    route.destination.y = 2.0
    route.destination.z = 3.0

    # Create a TransformStamped object
    tf = TransformStamped()
    tf.transform.translation.x = 10.0
    tf.transform.translation.y = 20.0
    tf.transform.translation.z = 30.0
    tf.transform.rotation.x = 0.0
    tf.transform.rotation.y = 0.0
    tf.transform.rotation.z = 1.0
    tf.transform.rotation.w = 0.0

    # Perform the transformation
    route_tf = do_transform_route(route, tf)

    # Assert the transformed route
    assert math.isclose(route_tf.destination.x, 9.0, abs_tol=EPS)
    assert math.isclose(route_tf.destination.y, 18.0, abs_tol=EPS)
    assert math.isclose(route_tf.destination.z, 33.0, abs_tol=EPS)

if __name__ == "__main__":
    pytest.main()