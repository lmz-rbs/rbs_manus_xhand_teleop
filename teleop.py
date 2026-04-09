#!/usr/bin/env python3
"""
Manus -> GeoRT -> XHand Teleoperation Pipeline

End-to-end script that:
  1. Reads human hand keypoints from Manus glove (via ZMQ or SDK)
  2. Retargets to XHand joint angles via trained GeoRT model
  3. Sends commands to physical XHand (or simulated)
  4. Optionally shows SAPIEN visualization

Usage:
  # Simulation mode (no hardware):
  python teleop.py --mocap sim --hand sim

  # With Manus glove via ZMQ + simulated hand:
  python teleop.py --mocap zmq --hand sim --viz

  # Full hardware:
  python teleop.py --mocap zmq --ckpt xhand_geort_v1 --hand-port /dev/ttyUSB0

  # With SAPIEN visualization:
  python teleop.py --mocap zmq --ckpt xhand_geort_v1 --hand-port /dev/ttyUSB0 --viz
"""

import sys
import os
import time
import numpy as np
import logging
import argparse
import signal
from pathlib import Path

# Add project src to path
PROJECT_ROOT = Path(__file__).parent.resolve()
SRC_DIR = PROJECT_ROOT / "src"
sys.path.insert(0, str(SRC_DIR))

# Also add GeoRT
GEORT_ROOT = PROJECT_ROOT / "third_party" / "GeoRT"
sys.path.insert(0, str(GEORT_ROOT))

from manus_bridge import ManusGloveBridge
from xhand_controller import XHandController, XHandSimController, create_controller

logger = logging.getLogger(__name__)


class TeleOpPipeline:
    """
    End-to-end teleoperation pipeline.
    
    Manus Glove -> (21,3) keypoints -> GeoRT Model -> (12,) qpos -> XHand
    """

    def __init__(self, mocap_mode="zmq", ckpt_tag=None, 
                 hand_sim=False, hand_port="/dev/ttyUSB0",
                 enable_viz=False, rate_hz=50,
                 smoothing_alpha=0.3, zmq_port=8765):
        """
        Args:
            mocap_mode: "zmq" | "integrated" | "sim"
            ckpt_tag: GeoRT checkpoint tag. If None, uses direct IK (no learned model)
            hand_sim: If True, simulate XHand instead of hardware
            hand_port: Serial port for real XHand
            enable_viz: Show SAPIEN visualization
            rate_hz: Control loop rate
            smoothing_alpha: Exponential smoothing factor (0=no smoothing, 1=no filtering)
            zmq_port: ZMQ port for mocap data
        """
        self.rate_hz = rate_hz
        self.smoothing_alpha = smoothing_alpha
        self.enable_viz = enable_viz
        self._running = False
        self._prev_qpos = None
        
        # 1. Mocap bridge
        logger.info(f"Initializing mocap bridge: mode={mocap_mode}")
        self.mocap = ManusGloveBridge(mode=mocap_mode, port=zmq_port)
        
        # 2. Retargeting model
        if ckpt_tag:
            logger.info(f"Loading GeoRT model: {ckpt_tag}")
            from retargeting import XHandRetargeter
            self.retargeter = XHandRetargeter.load(ckpt_tag)
        else:
            logger.info("No checkpoint specified - using XHandRetargeter without model")
            self.retargeter = None
        
        # 3. XHand controller
        if hand_sim:
            logger.info("Using simulated XHand controller")
            self.hand = XHandSimController()
        else:
            logger.info(f"Connecting to XHand on {hand_port}")
            self.hand = create_controller(sim=False, port=hand_port)
        
        # 4. Visualization (optional)
        self.viewer_env = None
        self.hand_model = None
        if enable_viz:
            self._setup_visualization()
    
    def _setup_visualization(self):
        """Initialize SAPIEN visualization."""
        try:
            from retargeting import load_xhand_config
            from geort.env.hand import HandKinematicModel
            
            config = load_xhand_config()
            self.hand_model = HandKinematicModel.build_from_config(config, render=True)
            self.viewer_env = self.hand_model.get_viewer_env()
            logger.info("SAPIEN visualization initialized")
        except Exception as e:
            logger.warning(f"Could not initialize visualization: {e}")
            self.enable_viz = False
    
    def _smooth_qpos(self, qpos):
        """Apply exponential smoothing to reduce jitter."""
        if self._prev_qpos is None:
            self._prev_qpos = qpos.copy()
            return qpos
        
        smoothed = self.smoothing_alpha * qpos + (1 - self.smoothing_alpha) * self._prev_qpos
        self._prev_qpos = smoothed.copy()
        return smoothed

    def step(self):
        """Execute one step of the teleoperation pipeline."""
        # 1. Get mocap data
        data = self.mocap.get()
        
        if data["status"] != "recording" or data["result"] is None:
            return False
        
        keypoints = data["result"]  # (21, 3)
        
        # 2. Retarget
        if self.retargeter is not None:
            qpos = self.retargeter.forward(keypoints)  # (12,)
        else:
            # Without a trained model, we cannot retarget.
            # This branch is for visualization-only or data collection.
            logger.debug("No retargeting model - skipping hand control")
            return True
        
        # 3. Smooth
        qpos = self._smooth_qpos(qpos)
        
        # 4. Send to hand
        self.hand.send_joint_positions(qpos)
        
        # 5. Update visualization
        if self.enable_viz and self.hand_model is not None:
            self.hand_model.set_qpos_target(qpos)
            self.viewer_env.update()
        
        return True

    def run(self):
        """Main teleoperation loop."""
        self._running = True
        dt = 1.0 / self.rate_hz
        
        logger.info("=" * 60)
        logger.info("  Manus -> GeoRT -> XHand Teleoperation")
        logger.info(f"  Rate: {self.rate_hz} Hz | Smoothing: {self.smoothing_alpha}")
        logger.info(f"  Visualization: {'ON' if self.enable_viz else 'OFF'}")
        logger.info("  Press Ctrl+C to stop")
        logger.info("=" * 60)
        
        step_count = 0
        active_count = 0
        start_time = time.time()
        
        try:
            while self._running:
                t0 = time.time()
                
                active = self.step()
                step_count += 1
                if active:
                    active_count += 1
                
                # Status print every 5 seconds
                if step_count % (self.rate_hz * 5) == 0:
                    elapsed = time.time() - start_time
                    logger.info(
                        f"Steps: {step_count} | Active: {active_count} | "
                        f"Elapsed: {elapsed:.1f}s | "
                        f"Effective Hz: {active_count / elapsed:.1f}"
                    )
                
                # Rate limiting
                elapsed = time.time() - t0
                sleep_time = dt - elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)
                    
        except KeyboardInterrupt:
            logger.info("Teleoperation stopped by user")
        finally:
            self.stop()
    
    def stop(self):
        """Clean shutdown."""
        self._running = False
        
        logger.info("Shutting down...")
        
        # Return hand to safe position
        try:
            safe_qpos = np.zeros(12)
            self.hand.send_joint_positions(safe_qpos)
            time.sleep(0.5)
        except Exception:
            pass
        
        self.hand.disconnect()
        self.mocap.close()
        
        logger.info("Teleoperation pipeline shut down cleanly")


def main():
    parser = argparse.ArgumentParser(
        description="Manus -> GeoRT -> XHand Teleoperation",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Test with simulated mocap + simulated hand + visualization:
  python teleop.py --mocap sim --hand sim --viz

  # Manus ZMQ + simulated hand (test retargeting):
  python teleop.py --mocap zmq --ckpt xhand_geort_v1 --hand sim --viz

  # Full hardware pipeline:
  python teleop.py --mocap zmq --ckpt xhand_geort_v1 --hand-port /dev/ttyUSB0 --viz
        """
    )
    
    # Mocap
    parser.add_argument("--mocap", type=str, default="sim",
                        choices=["zmq", "integrated", "sim", "auto"],
                        help="Mocap data source mode")
    parser.add_argument("--zmq-port", type=int, default=8765,
                        help="ZMQ port for mocap data")
    
    # Retargeting
    parser.add_argument("--ckpt", type=str, default=None,
                        help="GeoRT checkpoint tag (e.g., 'xhand_geort_v1')")
    
    # XHand
    parser.add_argument("--hand", type=str, default="sim",
                        choices=["sim", "real"],
                        help="XHand controller mode")
    parser.add_argument("--hand-port", type=str, default="/dev/ttyUSB0",
                        help="XHand serial port")
    
    # Visualization & control
    parser.add_argument("--viz", action="store_true",
                        help="Enable SAPIEN visualization")
    parser.add_argument("--rate", type=int, default=50,
                        help="Control loop rate (Hz)")
    parser.add_argument("--smoothing", type=float, default=0.3,
                        help="Exponential smoothing alpha (0-1, higher=less smooth)")
    
    # Logging
    parser.add_argument("--debug", action="store_true",
                        help="Enable debug logging")
    
    args = parser.parse_args()
    
    # Setup logging
    log_level = logging.DEBUG if args.debug else logging.INFO
    logging.basicConfig(
        level=log_level,
        format="%(asctime)s [%(name)s] %(levelname)s: %(message)s",
        datefmt="%H:%M:%S"
    )
    
    # Handle Ctrl+C gracefully
    pipeline = None
    
    def signal_handler(sig, frame):
        if pipeline:
            pipeline.stop()
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    
    # Create and run pipeline
    pipeline = TeleOpPipeline(
        mocap_mode=args.mocap,
        ckpt_tag=args.ckpt,
        hand_sim=(args.hand == "sim"),
        hand_port=args.hand_port,
        enable_viz=args.viz,
        rate_hz=args.rate,
        smoothing_alpha=args.smoothing,
        zmq_port=args.zmq_port,
    )
    
    pipeline.run()


if __name__ == "__main__":
    main()
