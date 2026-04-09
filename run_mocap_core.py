#!/usr/bin/env python3
"""Launch manus_mocap_core (ROS2 -> FK -> ZMQ broadcast)."""
import sys, os

# mocap_core only needs rclpy, scipy, numpy, zmq - no SAPIEN needed
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "third_party", "GeoRT"))

from geort.mocap.manus_mocap_core import main
main()
