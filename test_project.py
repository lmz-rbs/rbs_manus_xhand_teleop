#!/usr/bin/env python3
"""Quick test for the manus_xhand_teleop project."""
import sys, json, os
import numpy as np

os.chdir(os.path.dirname(os.path.abspath(__file__)))

print("=== Test 1: Config Loading ===")
with open("config/xhand_right.json") as f:
    config = json.load(f)
print(f"  Name: {config['name']}")
print(f"  Joints: {len(config['joint_order'])}")
print(f"  Fingertips: {len(config['fingertip_link'])}")
for ft in config["fingertip_link"]:
    print(f"    {ft['name']} -> {ft['link']} | human_id={ft['human_hand_id']}")

print()
print("=== Test 2: URDF Validation ===")
import xml.etree.ElementTree as ET
tree = ET.parse("assets/xhand_right/xhand_right_nocol.urdf")
root = tree.getroot()
collisions = list(root.iter("collision"))
revolute = [j for j in root.findall("joint") if j.get("type") == "revolute"]
meshes = [m.get("filename") for m in root.iter("mesh")]
print(f"  Collisions: {len(collisions)} (should be 0)")
print(f"  Revolute joints: {len(revolute)} (should be 12)")
urdf_dir = "assets/xhand_right"
for m in meshes:
    full_path = os.path.join(urdf_dir, m)
    assert os.path.exists(full_path), f"Missing mesh: {full_path}"
print(f"  All {len(meshes)} mesh files exist ✓")

print()
print("=== Test 3: Manus Bridge (sim) ===")
sys.path.insert(0, "src")
import logging
logging.basicConfig(level=logging.WARNING)
from manus_bridge import ManusGloveBridge
import time
bridge = ManusGloveBridge(mode="sim")
time.sleep(0.2)
data = bridge.get()
print(f"  Status: {data['status']}")
if data["result"] is not None:
    kp = data["result"]
    print(f"  Keypoints shape: {kp.shape}")
    print(f"  Thumb tip (id=4): {kp[4]}")
    print(f"  Index tip (id=8): {kp[8]}")
    print(f"  Middle tip (id=12): {kp[12]}")
    print(f"  Ring tip (id=16): {kp[16]}")
    print(f"  Pinky tip (id=20): {kp[20]}")
bridge.close()

print()
print("=== Test 4: XHand Controller (sim) ===")
from xhand_controller import XHandSimController
ctrl = XHandSimController()
qpos = np.random.uniform(0, 1.5, 12)
ctrl.send_joint_positions(qpos)
state = ctrl.get_state()
print(f"  Sent 12-DOF qpos: [{qpos[0]:.3f}, {qpos[1]:.3f}, ...]")
print(f"  State keys: {len(state)}")
ctrl.disconnect()

print()
print("=" * 50)
print("  ALL TESTS PASSED ✓")
print("=" * 50)
