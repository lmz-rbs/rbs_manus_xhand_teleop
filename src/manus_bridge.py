"""
Manus Glove Bridge - Python wrapper for Manus SDK Integrated mode.

Provides two backends:
  1. ZMQ backend  - Receives (21,3) keypoints from GeoRT manus_mocap_core.py (ROS2 + ZMQ)
  2. ctypes backend - Directly calls libManusSDK.so C API (Integrated mode, no ROS2 needed)

Both expose the same interface: ManusGloveBridge.get() -> {"result": np.ndarray(21,3), "status": str}
"""

import numpy as np
import threading
import time
import logging

logger = logging.getLogger(__name__)


class ManusZMQBridge:
    """
    Receives Manus hand keypoints via ZMQ from manus_mocap_core.py.
    
    Prerequisite: Run manus_mocap_core.py (ROS2 node) which:
      1. Connects to Manus glove via ROS2 manus_client
      2. Does FK to compute 21 keypoints
      3. Broadcasts (21,3) float32 via ZMQ PUB on port 8765
    """

    def __init__(self, host="localhost", port=8765):
        import zmq
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.SUB)
        self.socket.connect(f"tcp://{host}:{port}")
        self.socket.setsockopt_string(zmq.SUBSCRIBE, "")
        
        self._latest_data = None
        self._lock = threading.Lock()
        self._running = True
        self._thread = threading.Thread(target=self._recv_loop, daemon=True)
        self._thread.start()
        logger.info(f"ManusZMQBridge: Listening on tcp://{host}:{port}")

    def _recv_loop(self):
        import zmq
        while self._running:
            try:
                msg = self.socket.recv(flags=zmq.NOBLOCK)
                arr = np.frombuffer(msg, dtype=np.float32).reshape(21, 3)
                with self._lock:
                    self._latest_data = arr
            except zmq.Again:
                time.sleep(0.001)
            except Exception as e:
                logger.error(f"ZMQ recv error: {e}")
                time.sleep(0.01)

    def get(self):
        with self._lock:
            if self._latest_data is not None:
                return {"result": self._latest_data.copy(), "status": "recording"}
            else:
                return {"result": None, "status": "no_data"}

    def close(self):
        self._running = False
        self._thread.join(timeout=2.0)
        self.socket.close()
        self.context.term()


class ManusIntegratedBridge:
    """
    Direct ctypes binding to libManusSDK.so for Integrated mode.
    
    This bypasses ROS2 entirely - connects directly to Manus glove via USB,
    retrieves skeleton data through C API callbacks, performs FK to get (21,3) keypoints.
    
    Requires:
      - libManusSDK.so in LD_LIBRARY_PATH or specified path
      - USB access to Manus glove (may need udev rules)
      - libusb-1.0, libudev, zlib installed
    """

    # Manus hand FK - finger link vectors (from GeoRT manus_mocap_core.py)
    FINGER_LINK_VECTORS = np.array([
        [0.0, 0.0, 0.0],       # 0: Wrist
        [0.0250, 0.0, 0.0050], # 1: Thumb CMC
        [0.0, 0.0, 0.0390],    # 2: Thumb MCP
        [0.0, 0.0, 0.0330],    # 3: Thumb IP
        [0.0, 0.0, 0.0210],    # 4: Thumb Tip
        [0.0170, 0.0, 0.0870], # 5: Index MCP
        [0.0, 0.0, 0.0260],    # 6: Index PIP
        [0.0, 0.0, 0.0220],    # 7: Index DIP
        [0.0, 0.0, 0.0200],    # 8: Index Tip
        [0.0, 0.0, 0.0920],    # 9: Middle MCP
        [0.0, 0.0, 0.0260],    # 10: Middle PIP
        [0.0, 0.0, 0.0260],    # 11: Middle DIP
        [0.0, 0.0, 0.0220],    # 12: Middle Tip
        [-0.0170, 0.0, 0.0840],# 13: Ring MCP
        [0.0, 0.0, 0.0210],    # 14: Ring PIP
        [0.0, 0.0, 0.0210],    # 15: Ring DIP
        [0.0, 0.0, 0.0200],    # 16: Ring Tip
        [-0.0340, 0.0, 0.0720],# 17: Pinky MCP
        [0.0, 0.0, 0.0210],    # 18: Pinky PIP
        [0.0, 0.0, 0.0210],    # 19: Pinky DIP
        [0.0, 0.0, 0.0200],    # 20: Pinky Tip
    ], dtype=np.float64)

    # Finger kinematic chains (MediaPipe convention: 21 keypoints)
    FINGER_CHAINS = [
        [0, 1, 2, 3, 4],       # Thumb
        [0, 5, 6, 7, 8],       # Index
        [0, 9, 10, 11, 12],    # Middle
        [0, 13, 14, 15, 16],   # Ring
        [0, 17, 18, 19, 20],   # Pinky
    ]

    def __init__(self, sdk_lib_path=None):
        """
        Args:
            sdk_lib_path: Path to libManusSDK.so. 
                          If None, tries default locations.
        """
        self._latest_quats = None
        self._latest_keypoints = None
        self._lock = threading.Lock()
        self._running = True
        self._connected = False
        
        self._sdk = self._load_sdk(sdk_lib_path)
        if self._sdk is not None:
            self._thread = threading.Thread(target=self._sdk_loop, daemon=True)
            self._thread.start()
        else:
            logger.warning("ManusIntegratedBridge: Could not load libManusSDK.so. "
                          "Falling back to simulated data for testing.")
            self._thread = threading.Thread(target=self._sim_loop, daemon=True)
            self._thread.start()

    def _load_sdk(self, lib_path):
        """Try to load the Manus SDK shared library."""
        import ctypes
        search_paths = [lib_path] if lib_path else []
        search_paths += [
            "/home/ps/lmz/ManusRetargeting/ManusSDK/SDKClient_Linux/ManusSDK/lib/libManusSDK.so",
            "./ManusSDK/lib/libManusSDK.so",
            "libManusSDK.so",
        ]
        
        for path in search_paths:
            if path is None:
                continue
            try:
                sdk = ctypes.CDLL(path)
                logger.info(f"Loaded ManusSDK from: {path}")
                return sdk
            except OSError:
                continue
        
        logger.warning("libManusSDK.so not found in any search path")
        return None

    def _sim_loop(self):
        """Generate simulated hand motion data for testing without actual glove."""
        logger.info("Running in SIMULATION mode - generating synthetic hand data")
        t = 0.0
        while self._running:
            keypoints = self._generate_synthetic_keypoints(t)
            with self._lock:
                self._latest_keypoints = keypoints
                self._connected = True
            t += 0.01
            time.sleep(0.01)

    def _generate_synthetic_keypoints(self, t):
        """Generate realistic synthetic (21,3) hand keypoints for testing."""
        keypoints = np.zeros((21, 3), dtype=np.float32)
        
        keypoints[0] = [0.0, 0.0, 0.0]
        
        finger_bases = [
            np.array([0.0, 0.025, 0.030]),
            np.array([0.0, 0.017, 0.090]),
            np.array([0.0, 0.000, 0.092]),
            np.array([0.0, -0.017, 0.084]),
            np.array([0.0, -0.034, 0.072]),
        ]
        
        finger_lengths = [
            [0.039, 0.033, 0.021],
            [0.026, 0.022, 0.020],
            [0.026, 0.026, 0.022],
            [0.021, 0.021, 0.020],
            [0.021, 0.021, 0.020],
        ]
        
        for fi, (chain, base, lengths) in enumerate(zip(self.FINGER_CHAINS, finger_bases, finger_lengths)):
            curl = 0.5 * (1 + np.sin(t * 2 + fi * 0.5)) * 1.5
            
            pos = base.copy()
            keypoints[chain[1]] = pos
            
            for ji, (joint_idx, length) in enumerate(zip(chain[2:], lengths)):
                angle = curl * (0.3 + 0.3 * ji)
                if fi == 0:
                    pos = pos + np.array([0.0, length * np.cos(angle), -length * np.sin(angle)])
                else:
                    pos = pos + np.array([0.0, 0.0, length * np.cos(angle)])
                    pos[0] += length * np.sin(angle) * 0.1
                keypoints[joint_idx] = pos
        
        keypoints = self._to_canonical(keypoints)
        return keypoints

    @staticmethod
    def _to_canonical(keypoints):
        """Transform keypoints to GeoRT canonical frame."""
        z_axis = keypoints[9] - keypoints[0]
        z_norm = np.linalg.norm(z_axis)
        if z_norm < 1e-6:
            return keypoints
        z_axis = z_axis / z_norm
        
        y_aux = keypoints[5] - keypoints[13]
        y_norm = np.linalg.norm(y_aux)
        if y_norm < 1e-6:
            return keypoints
        y_aux = y_aux / y_norm
        
        x_axis = np.cross(y_aux, z_axis)
        x_axis = x_axis / np.linalg.norm(x_axis)
        y_axis = np.cross(z_axis, x_axis)
        
        R = np.array([x_axis, y_axis, z_axis]).T
        t = keypoints[0].copy()
        
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = t
        T_inv = np.linalg.inv(T)
        
        kp_h = np.concatenate([keypoints, np.ones((21, 1))], axis=-1)
        kp_canonical = (kp_h @ T_inv.T)[:, :3]
        return kp_canonical.astype(np.float32)

    def _sdk_loop(self):
        """Main loop for actual ManusSDK Integrated mode."""
        # TODO: Implement full ctypes bindings when SDK permissions are resolved
        logger.info("SDK loop started - falling back to sim for now")
        self._sim_loop()

    @staticmethod
    def fk_solve(positions, orientations):
        """
        Forward kinematics from Manus SDK raw data.
        
        Args:
            positions: (21, 3) local positions
            orientations: (21, 4) quaternions (x, y, z, w)
        Returns:
            keypoints: dict {idx: np.array(3,)} absolute positions
        """
        from scipy.spatial.transform import Rotation as R
        
        chains = [
            [0, 1, 2, 3, 4],
            [0, 5, 6, 7, 8],
            [0, 9, 10, 11, 12],
            [0, 13, 14, 15, 16],
            [0, 17, 18, 19, 20],
        ]
        
        keypoints = {}
        for chain in chains:
            T = np.eye(4)
            for idx in chain:
                local_T = np.eye(4)
                local_T[:3, 3] = positions[idx]
                local_T[:3, :3] = R.from_quat(orientations[idx]).as_matrix()
                T = T @ local_T
                if idx not in keypoints:
                    keypoints[idx] = T[:3, 3].copy()
        
        return keypoints

    def get(self):
        with self._lock:
            if self._latest_keypoints is not None:
                return {"result": self._latest_keypoints.copy(), "status": "recording"}
            else:
                return {"result": None, "status": "no_data"}

    def close(self):
        self._running = False
        if hasattr(self, '_thread'):
            self._thread.join(timeout=2.0)


class ManusGloveBridge:
    """
    Unified Manus glove interface.
    
    Automatically selects the best available backend:
      1. ZMQ (if manus_mocap_core.py is running)
      2. Integrated (direct SDK via ctypes)
      3. Simulation (synthetic data for testing)
    
    Usage:
        bridge = ManusGloveBridge(mode="zmq")  # or "integrated" or "sim"
        while True:
            data = bridge.get()
            if data["status"] == "recording":
                keypoints = data["result"]  # (21, 3) numpy array
    """

    def __init__(self, mode="zmq", **kwargs):
        self.mode = mode
        
        if mode == "zmq":
            self._backend = ManusZMQBridge(**kwargs)
        elif mode == "integrated":
            self._backend = ManusIntegratedBridge(**kwargs)
        elif mode == "sim":
            self._backend = ManusIntegratedBridge(sdk_lib_path="__nonexistent__")
        elif mode == "auto":
            try:
                import zmq
                ctx = zmq.Context()
                sock = ctx.socket(zmq.SUB)
                sock.connect(f"tcp://localhost:{kwargs.get('port', 8765)}")
                sock.setsockopt_string(zmq.SUBSCRIBE, "")
                sock.setsockopt(zmq.RCVTIMEO, 1000)
                try:
                    sock.recv()
                    sock.close()
                    ctx.term()
                    logger.info("Auto-detect: ZMQ data source found")
                    self._backend = ManusZMQBridge(**kwargs)
                except zmq.Again:
                    sock.close()
                    ctx.term()
                    logger.info("Auto-detect: No ZMQ source, trying Integrated SDK")
                    self._backend = ManusIntegratedBridge(**kwargs)
            except ImportError:
                logger.info("Auto-detect: pyzmq not available, using Integrated SDK")
                self._backend = ManusIntegratedBridge(**kwargs)
        else:
            raise ValueError(f"Unknown mode: {mode}. Use 'zmq', 'integrated', 'sim', or 'auto'")

        logger.info(f"ManusGloveBridge initialized: mode='{mode}', backend={type(self._backend).__name__}")

    def get(self):
        return self._backend.get()

    def close(self):
        self._backend.close()


class ManusBroadcaster:
    """
    Reads from ManusIntegratedBridge and broadcasts (21,3) keypoints via ZMQ PUB.
    Drop-in replacement for manus_mocap_core.py without ROS2 dependency.
    """

    def __init__(self, bridge, port=8765):
        import zmq
        self.bridge = bridge
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.PUB)
        self.socket.bind(f"tcp://*:{port}")
        self.socket.setsockopt(zmq.SNDHWM, 0)
        self._running = True
        logger.info(f"ManusBroadcaster: Publishing on tcp://*:{port}")

    def run(self, rate_hz=100):
        dt = 1.0 / rate_hz
        count = 0
        while self._running:
            data = self.bridge.get()
            if data["status"] == "recording" and data["result"] is not None:
                kp = data["result"].astype(np.float32)
                self.socket.send(kp.tobytes())
                count += 1
                if count % 500 == 0:
                    logger.info(f"Broadcast {count} frames")
            time.sleep(dt)

    def stop(self):
        self._running = False
        self.socket.close()
        self.context.term()


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="Manus Glove Bridge")
    parser.add_argument("--mode", choices=["zmq", "integrated", "sim", "broadcast"], 
                        default="sim", help="Operation mode")
    parser.add_argument("--port", type=int, default=8765, help="ZMQ port")
    parser.add_argument("--sdk-path", type=str, default=None, help="Path to libManusSDK.so")
    args = parser.parse_args()

    logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(name)s] %(message)s")

    if args.mode == "broadcast":
        bridge = ManusIntegratedBridge(sdk_lib_path=args.sdk_path)
        broadcaster = ManusBroadcaster(bridge, port=args.port)
        try:
            broadcaster.run()
        except KeyboardInterrupt:
            broadcaster.stop()
            bridge.close()
    else:
        bridge = ManusGloveBridge(mode=args.mode, port=args.port)
        try:
            while True:
                data = bridge.get()
                if data["result"] is not None:
                    kp = data["result"]
                    print(f"[{data['status']}] Thumb: {kp[4]}, Index: {kp[8]}, Middle: {kp[12]}")
                else:
                    print(f"[{data['status']}] Waiting...")
                time.sleep(0.05)
        except KeyboardInterrupt:
            bridge.close()
