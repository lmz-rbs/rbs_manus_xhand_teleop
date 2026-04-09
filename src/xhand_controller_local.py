"""
XHand Controller - Python wrapper for XHand dexterous hand control.

Maps GeoRT retargeted joint angles (12-DOF, radians) to XHand motor commands.

Joint order (matching xhand_right.json config):
  [0]  right_hand_thumb_bend_joint    -> motor 0
  [1]  right_hand_thumb_rota_joint1   -> motor 1
  [2]  right_hand_thumb_rota_joint2   -> motor 2
  [3]  right_hand_index_bend_joint    -> motor 3
  [4]  right_hand_index_joint1        -> motor 4
  [5]  right_hand_index_joint2        -> motor 5
  [6]  right_hand_mid_joint1          -> motor 6
  [7]  right_hand_mid_joint2          -> motor 7
  [8]  right_hand_ring_joint1         -> motor 8
  [9]  right_hand_ring_joint2         -> motor 9
  [10] right_hand_pinky_joint1        -> motor 10
  [11] right_hand_pinky_joint2        -> motor 11
"""

import numpy as np
import math
import time
import logging
from xhand_controller import xhand_control

logger = logging.getLogger(__name__)


class XHandController:
    """
    Controls the physical XHand via xhand_controller SDK.
    
    Usage:
        ctrl = XHandController(port="/dev/ttyUSB0")
        ctrl.connect()
        ctrl.send_joint_positions(qpos_rad)  # 12-dim numpy array in radians
        ctrl.disconnect()
    """
    
    # Joint limits (radians) - from URDF
    JOINT_LIMITS = {
        "right_hand_thumb_bend_joint":  (0.0, 1.832),
        "right_hand_thumb_rota_joint1": (-0.698, 1.57),
        "right_hand_thumb_rota_joint2": (0.0, 1.57),
        "right_hand_index_bend_joint":  (-0.174, 0.174),
        "right_hand_index_joint1":      (0.0, 1.919),
        "right_hand_index_joint2":      (0.0, 1.919),
        "right_hand_mid_joint1":        (0.0, 1.919),
        "right_hand_mid_joint2":        (0.0, 1.919),
        "right_hand_ring_joint1":       (0.0, 1.919),
        "right_hand_ring_joint2":       (0.0, 1.919),
        "right_hand_pinky_joint1":      (0.0, 1.919),
        "right_hand_pinky_joint2":      (0.0, 1.919),
    }
    
    JOINT_NAMES = [
        "right_hand_thumb_bend_joint",
        "right_hand_thumb_rota_joint1",
        "right_hand_thumb_rota_joint2",
        "right_hand_index_bend_joint",
        "right_hand_index_joint1",
        "right_hand_index_joint2",
        "right_hand_mid_joint1",
        "right_hand_mid_joint2",
        "right_hand_ring_joint1",
        "right_hand_ring_joint2",
        "right_hand_pinky_joint1",
        "right_hand_pinky_joint2",
    ]
    
    # Default PD gains
    DEFAULT_KP = 100
    DEFAULT_KI = 0
    DEFAULT_KD = 5
    DEFAULT_TOR_MAX = 500
    
    # Preset actions (degrees) from xhand SDK example
    PRESETS = {
        "fist":  [11.85, 74.58, 40, -3.08, 106.02, 110, 109.75, 107.56, 107.66, 110, 109.1, 109.15],
        "palm":  [0, 80.66, 33.2, 0.00, 5.11, 0, 6.53, 0, 6.76, 4.41, 10.13, 0],
        "v":     [38.32, 90, 52.08, 6.21, 2.6, 0, 2.1, 0, 110, 110, 110, 109.23],
        "ok":    [45.88, 41.54, 67.35, 2.22, 80.45, 70.82, 31.37, 10.39, 13.69, 16.88, 1.39, 10.55],
    }

    def __init__(self, port="/dev/ttyUSB0", hand_id=0, baudrate=3000000,
                 kp=None, ki=None, kd=None, tor_max=None):
        self.port = port
        self.hand_id = hand_id
        self.baudrate = baudrate
        self.kp = kp or self.DEFAULT_KP
        self.ki = ki or self.DEFAULT_KI
        self.kd = kd or self.DEFAULT_KD
        self.tor_max = tor_max or self.DEFAULT_TOR_MAX
        
        self._hand = None
        self._command = None
        self._connected = False
        
        # Build joint limit arrays
        self._lower = np.array([self.JOINT_LIMITS[n][0] for n in self.JOINT_NAMES])
        self._upper = np.array([self.JOINT_LIMITS[n][1] for n in self.JOINT_NAMES])

    def connect(self):
        """Connect to the XHand via serial port."""
        try:
            # import xhand_control
            # from xhand_controller import xhand_control
            print(11111)
            self._hand = xhand_control.XHandControl()
            rsp = self._hand.open_serial(self.port, self.baudrate); ret = rsp.error_code
            if ret != 0:
                logger.error(f"Failed to open XHand on {self.port}, return code: {ret}")
                return False
            print(22222)
            self._command = xhand_control.HandCommand_t()
            print(33333)
            # Set default PD gains for all motors
            for i in range(12):
                self._command.finger_command[i].id = int(i)
                self._command.finger_command[i].kp = int(self.kp)
                self._command.finger_command[i].ki = int(self.ki)
                self._command.finger_command[i].kd = int(self.kd)
                self._command.finger_command[i].tor_max = int(self.tor_max)
                self._command.finger_command[i].mode = 3  # Position mode
            
            # Set position mode
            print(44444)
            self._connected = True
            logger.info(f"XHand connected on {self.port}")
            return True
            
        except ImportError:
            logger.error("xhand_control module not found. Install: pip install xhand_controller-*.whl")
            return False
        except Exception as e:
            logger.error(f"XHand connection failed: {e}")
            return False

    def send_joint_positions(self, qpos_rad):
        """
        Send joint position commands to XHand.
        
        Args:
            qpos_rad: numpy array of shape (12,) - joint angles in radians.
                      Order matches JOINT_NAMES / xhand_right.json joint_order.
        """
        if not self._connected:
            logger.warning("XHand not connected!")
            return False
        
        # Clip to joint limits
        qpos_clipped = np.clip(qpos_rad, self._lower, self._upper)
        
        for i in range(12):
            self._command.finger_command[i].position = float(qpos_clipped[i])
        
        try:
            self._hand.send_command(self.hand_id, self._command)
            return True
        except Exception as e:
            logger.error(f"Failed to send command: {e}")
            return False

    def send_preset(self, name):
        """
        Send a preset action by name.
        
        Args:
            name: "fist", "palm", "v", or "ok"
        """
        if name not in self.PRESETS:
            logger.error(f"Unknown preset: {name}. Available: {list(self.PRESETS.keys())}")
            return False
        
        degrees = self.PRESETS[name]
        radians = np.array([d * math.pi / 180.0 for d in degrees])
        return self.send_joint_positions(radians)

    def get_state(self, force_update=True):
        """Read current hand state from hardware."""
        if not self._connected:
            return None
        try:
            states = {}
            for i in range(12):
                state = self._hand.read_state(finger_id=i, force_update=force_update)
                states[self.JOINT_NAMES[i]] = state
            return states
        except Exception as e:
            logger.error(f"Failed to read state: {e}")
            return None

    def disconnect(self):
        """Close connection to XHand."""
        if self._connected and self._hand is not None:
            try:
                self._hand.close_device()
            except Exception:
                pass
            self._connected = False
            logger.info("XHand disconnected")

    def __del__(self):
        self.disconnect()


class XHandSimController:
    """
    Simulated XHand controller for testing without hardware.
    Logs commands and tracks virtual joint state.
    """
    
    def __init__(self):
        self._qpos = np.zeros(12)
        self._connected = True
        logger.info("XHandSimController: Running in simulation mode")

    def connect(self):
        self._connected = True
        return True

    def send_joint_positions(self, qpos_rad):
        self._qpos = np.clip(qpos_rad, 0, 2.0)
        return True

    def send_preset(self, name):
        presets = XHandController.PRESETS
        if name in presets:
            radians = np.array([d * math.pi / 180.0 for d in presets[name]])
            return self.send_joint_positions(radians)
        return False

    def get_state(self, force_update=True):
        return {name: self._qpos[i] for i, name in enumerate(XHandController.JOINT_NAMES)}

    def disconnect(self):
        self._connected = False


def create_controller(sim=False, **kwargs):
    """
    Factory function to create the appropriate controller.
    
    Args:
        sim: If True, returns a simulated controller
        **kwargs: Passed to XHandController (port, hand_id, etc.)
    """
    if sim:
        return XHandSimController()
    else:
        ctrl = XHandController(**kwargs)
        if ctrl.connect():
            return ctrl
        else:
            logger.warning("Hardware connection failed, falling back to simulation")
            return XHandSimController()


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="XHand Controller Test")
    parser.add_argument("--port", type=str, default="/dev/ttyUSB0")
    parser.add_argument("--sim", action="store_true", help="Use simulated controller")
    parser.add_argument("--preset", type=str, default=None, 
                        help="Send preset action: fist, palm, v, ok")
    args = parser.parse_args()

    logging.basicConfig(level=logging.INFO)

    ctrl = create_controller(sim=args.sim, port=args.port)
    
    if args.preset:
        ctrl.send_preset(args.preset)
        print(f"Sent preset: {args.preset}")
        time.sleep(2)
    else:
        # Demo: sweep all joints
        print("Sweeping all joints...")
        for t in np.linspace(0, 2 * math.pi, 200):
            qpos = np.ones(12) * 0.5 * (1 + np.sin(t)) * 1.0
            ctrl.send_joint_positions(qpos)
            state = ctrl.get_state()
            if state:
                print(f"  qpos[0:4] = {qpos[:4]}")
            time.sleep(0.02)
    
    ctrl.disconnect()
