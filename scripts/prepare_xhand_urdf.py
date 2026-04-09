#!/usr/bin/env python3
"""
prepare_xhand_urdf.py - Automatically prepare XHand URDF for SAPIEN + GeoRT.

This script handles:
  1. Remove all <collision> elements (SAPIEN visual-only mode)
  2. Fix mesh paths: package://xxx/ -> relative paths (meshes/xxx.STL)
  3. Copy processed URDF + mesh assets to GeoRT assets/ directory
  4. Check & align base_link coordinate system with GeoRT convention
  5. Validate SAPIEN compatibility (optional, requires sapien installed)
  6. Generate a summary report

Usage:
  # Full processing (original URDF -> SAPIEN-ready output):
  python3 prepare_xhand_urdf.py \
    --input  "/home/ps/lmz/ManusRetargeting/XHAND1_URDF_ver 1.3/xhand1_right/urdf/xhand_right.urdf" \
    --mesh-src "/home/ps/lmz/ManusRetargeting/XHAND1_URDF_ver 1.3/xhand1_right/meshes/" \
    --output-dir "/home/ps/lmz/ManusRetargeting/manus_xhand_teleop/assets/xhand_right/" \
    --geort-assets "/home/ps/lmz/ManusRetargeting/GeoRT/assets/xhand_right/"

  # Validate an already-processed URDF:
  python3 prepare_xhand_urdf.py --validate-only \
    --input "/home/ps/lmz/ManusRetargeting/manus_xhand_teleop/assets/xhand_right/xhand_right_nocol.urdf"

  # Dry-run (show what would be done without writing):
  python3 prepare_xhand_urdf.py --dry-run \
    --input "/home/ps/lmz/ManusRetargeting/XHAND1_URDF_ver 1.3/xhand1_right/urdf/xhand_right.urdf"
"""

import xml.etree.ElementTree as ET
import os
import sys
import shutil
import argparse
import json
import logging
import copy
from pathlib import Path
from dataclasses import dataclass, field
from typing import List, Dict, Tuple, Optional

logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")
logger = logging.getLogger("prepare_xhand_urdf")


# ============================================================================
# GeoRT Coordinate System Convention
# ============================================================================
# base_link frame:
#   +Y : center of palm → thumb direction
#   +Z : center of palm → middle finger tip direction
#   +X : palm normal (pointing outward from the back of the hand)
#
# For XHand right_hand_link, from the URDF joint origins:
#   - Thumb CMC at xyz=(0.0095, 0.0228, 0.0305) => thumb is in +Y direction ✓
#   - Index MCP at xyz=(0.0065, 0.0265, 0.0899) => fingers extend in +Z ✓
#   - Middle MCP at xyz=(0.0065, 0.004, 0.1082) => middle finger furthest in +Z ✓
#   - Ring MCP at xyz=(0.0065, -0.016, 0.1052)
#   - Pinky MCP at xyz=(0.0065, -0.036, 0.1022)
# Conclusion: XHand coordinate system matches GeoRT convention exactly.
# No virtual base_link needed!
# ============================================================================


@dataclass
class URDFProcessingReport:
    """Holds all processing results for the summary report."""
    input_path: str = ""
    output_path: str = ""
    original_link_count: int = 0
    original_joint_count: int = 0
    revolute_joint_count: int = 0
    fixed_joint_count: int = 0
    collisions_removed: int = 0
    mesh_paths_fixed: int = 0
    mesh_files_total: int = 0
    mesh_files_found: int = 0
    mesh_files_missing: List[str] = field(default_factory=list)
    base_link_name: str = ""
    coordinate_system_ok: bool = False
    coordinate_system_notes: str = ""
    sapien_compatible: Optional[bool] = None
    sapien_notes: str = ""
    virtual_base_added: bool = False
    warnings: List[str] = field(default_factory=list)
    errors: List[str] = field(default_factory=list)

    def print_report(self):
        """Print a formatted summary report."""
        print("\n" + "=" * 70)
        print("  XHand URDF Processing Report")
        print("=" * 70)
        print(f"  Input:  {self.input_path}")
        print(f"  Output: {self.output_path}")
        print(f"  Links:  {self.original_link_count}")
        print(f"  Joints: {self.original_joint_count} "
              f"(revolute: {self.revolute_joint_count}, fixed: {self.fixed_joint_count})")
        print()

        print("  Processing Steps:")
        print(f"    [{'✓' if self.collisions_removed > 0 else '—'}] Collisions removed: {self.collisions_removed}")
        print(f"    [{'✓' if self.mesh_paths_fixed > 0 else '—'}] Mesh paths fixed: {self.mesh_paths_fixed}")
        print(f"    [{'✓' if self.mesh_files_found == self.mesh_files_total else '✗'}] "
              f"Mesh files: {self.mesh_files_found}/{self.mesh_files_total} found")
        print(f"    [{'✓' if self.coordinate_system_ok else '✗'}] Coordinate system: "
              f"{self.coordinate_system_notes}")
        if self.virtual_base_added:
            print(f"    [✓] Virtual base_link added for coordinate alignment")
        
        if self.sapien_compatible is not None:
            status = '✓' if self.sapien_compatible else '✗'
            print(f"    [{status}] SAPIEN load test: {self.sapien_notes}")
        else:
            print(f"    [—] SAPIEN load test: skipped (sapien not installed)")

        if self.warnings:
            print()
            print("  ⚠ Warnings:")
            for w in self.warnings:
                print(f"    - {w}")

        if self.errors:
            print()
            print("  ✗ Errors:")
            for e in self.errors:
                print(f"    - {e}")

        if self.mesh_files_missing:
            print()
            print("  Missing mesh files:")
            for m in self.mesh_files_missing:
                print(f"    - {m}")

        overall = "PASS ✓" if (not self.errors and self.mesh_files_found == self.mesh_files_total 
                                and self.coordinate_system_ok) else "FAIL ✗"
        print()
        print(f"  Overall: {overall}")
        print("=" * 70)


def parse_urdf(urdf_path: str) -> ET.ElementTree:
    """Parse a URDF file."""
    tree = ET.parse(urdf_path)
    return tree


def count_elements(root: ET.Element) -> Dict[str, int]:
    """Count key URDF elements."""
    links = root.findall("link")
    joints = root.findall("joint")
    revolute = [j for j in joints if j.get("type") == "revolute"]
    fixed = [j for j in joints if j.get("type") == "fixed"]
    collisions = list(root.iter("collision"))
    visuals = list(root.iter("visual"))
    return {
        "links": len(links),
        "joints": len(joints),
        "revolute": len(revolute),
        "fixed": len(fixed),
        "collisions": len(collisions),
        "visuals": len(visuals),
    }


def remove_collisions(root: ET.Element) -> int:
    """Remove all <collision> elements from all links. Returns count removed."""
    removed = 0
    for link in root.findall("link"):
        for col in link.findall("collision"):
            link.remove(col)
            removed += 1
    return removed


def fix_mesh_paths(root: ET.Element, mesh_prefix: str = "meshes/") -> int:
    """
    Fix mesh filenames: replace package://xxx/ with relative path.
    
    SAPIEN resolves mesh paths relative to the URDF file's directory.
    So 'meshes/foo.STL' means <urdf_dir>/meshes/foo.STL.
    
    Returns count of paths fixed.
    """
    fixed = 0
    for mesh in root.iter("mesh"):
        fn = mesh.get("filename")
        if fn is None:
            continue
        
        original = fn
        
        # Handle package:// paths
        if "package://" in fn:
            # Extract just the filename after meshes/
            # e.g. "package://xhand_right/meshes/right_hand_link.STL" -> "meshes/right_hand_link.STL"
            parts = fn.split("/")
            # Find 'meshes' in the path and take everything from there
            try:
                meshes_idx = parts.index("meshes")
                fn = "/".join(parts[meshes_idx:])
            except ValueError:
                # No 'meshes' directory in path, just use the filename
                fn = mesh_prefix + parts[-1]
            mesh.set("filename", fn)
            fixed += 1
        
        # Handle absolute paths
        elif os.path.isabs(fn):
            basename = os.path.basename(fn)
            fn = mesh_prefix + basename
            mesh.set("filename", fn)
            fixed += 1
        
        # Already relative - ensure it starts with meshes/
        elif not fn.startswith("meshes/") and not fn.startswith("./meshes/"):
            # Path like "right_hand_link.STL" -> "meshes/right_hand_link.STL"
            if "/" not in fn:
                fn = mesh_prefix + fn
                mesh.set("filename", fn)
                fixed += 1
        
        if fn != original:
            logger.debug(f"  Fixed mesh path: {original} -> {fn}")
    
    return fixed


def get_mesh_filenames(root: ET.Element) -> List[str]:
    """Extract all unique mesh filenames from URDF."""
    filenames = set()
    for mesh in root.iter("mesh"):
        fn = mesh.get("filename")
        if fn:
            filenames.add(fn)
    return sorted(filenames)


def verify_mesh_files(mesh_filenames: List[str], urdf_dir: str) -> Tuple[int, int, List[str]]:
    """Check that all mesh files exist relative to URDF directory."""
    found = 0
    missing = []
    for fn in mesh_filenames:
        full_path = os.path.join(urdf_dir, fn)
        if os.path.exists(full_path):
            found += 1
        else:
            missing.append(fn)
    return found, len(mesh_filenames), missing


def check_coordinate_system(root: ET.Element) -> Tuple[bool, str]:
    """
    Check if the base_link coordinate system matches GeoRT convention:
      +Y: palm -> thumb
      +Z: palm -> middle finger
      +X: palm normal
    
    We do this by analyzing the joint origins from base_link to each finger.
    """
    # Find the base link name (first link in URDF)
    base_link = root.findall("link")[0].get("name")
    
    # Collect joint origins from base_link to first joint of each finger
    finger_origins = {}
    for joint in root.findall("joint"):
        parent = joint.find("parent").get("link")
        if parent == base_link:
            child = joint.find("child").get("link")
            origin = joint.find("origin")
            if origin is not None:
                xyz = origin.get("xyz", "0 0 0")
                x, y, z = [float(v) for v in xyz.split()]
                name = joint.get("name")
                finger_origins[name] = {"x": x, "y": y, "z": z, "child": child}
    
    if not finger_origins:
        return False, "No joints from base_link found - cannot verify coordinate system"
    
    # Analyze: find thumb (should be in +Y), fingers (should extend in +Z)
    notes = []
    thumb_found = False
    fingers_in_z = True
    
    for jname, origin in finger_origins.items():
        lower_name = jname.lower()
        
        if "thumb" in lower_name:
            thumb_found = True
            # Thumb should be in +Y direction (or at least Y > 0)
            if origin["y"] > 0:
                notes.append(f"Thumb ({jname}): Y={origin['y']:.4f} > 0 ✓ (toward +Y)")
            else:
                notes.append(f"Thumb ({jname}): Y={origin['y']:.4f} ✗ (expected +Y)")
                fingers_in_z = False
        
        elif any(f in lower_name for f in ["index", "mid", "ring", "pinky"]):
            # Fingers should extend mainly in +Z
            if origin["z"] > 0.05:  # At least 5cm in Z direction
                notes.append(f"Finger ({jname}): Z={origin['z']:.4f} > 0.05 ✓ (extends in +Z)")
            else:
                notes.append(f"Finger ({jname}): Z={origin['z']:.4f} ✗ (expected +Z > 0.05)")
                fingers_in_z = False
    
    # Also check: index/ring should differ primarily in Y (spread across Y axis)
    index_y = None
    pinky_y = None
    for jname, origin in finger_origins.items():
        if "index" in jname.lower():
            index_y = origin["y"]
        if "pinky" in jname.lower():
            pinky_y = origin["y"]
    
    if index_y is not None and pinky_y is not None:
        if index_y > pinky_y:
            notes.append(f"Index Y={index_y:.4f} > Pinky Y={pinky_y:.4f} ✓ (correct Y ordering)")
        else:
            notes.append(f"Index Y={index_y:.4f} <= Pinky Y={pinky_y:.4f} ✗ (unexpected Y ordering)")
            fingers_in_z = False
    
    is_ok = thumb_found and fingers_in_z
    summary = "GeoRT-compatible" if is_ok else "NEEDS ALIGNMENT"
    detail = "; ".join(notes)
    
    return is_ok, f"{summary} ({detail})"


def add_virtual_base_link(root: ET.Element, original_base: str, 
                          rotation_rpy: str = "0 0 0") -> str:
    """
    Add a virtual base_link with a fixed joint to align coordinate system.
    
    This is only needed if the original base_link doesn't match GeoRT convention.
    For XHand right hand, this is NOT needed (already aligned).
    
    Args:
        root: URDF XML root element
        original_base: Name of the original base link
        rotation_rpy: Roll-Pitch-Yaw rotation to apply
    
    Returns:
        Name of the new virtual base link
    """
    virtual_name = "geort_base_link"
    
    # Create virtual base link (minimal inertia)
    virtual_link = ET.SubElement(root, "link")
    virtual_link.set("name", virtual_name)
    inertial = ET.SubElement(virtual_link, "inertial")
    ET.SubElement(inertial, "mass", value="0.001")
    ET.SubElement(inertial, "origin", xyz="0 0 0", rpy="0 0 0")
    ET.SubElement(inertial, "inertia", ixx="0.000001", ixy="0", ixz="0",
                  iyy="0.000001", iyz="0", izz="0.000001")
    
    # Create fixed joint from virtual base to original base
    virtual_joint = ET.SubElement(root, "joint")
    virtual_joint.set("name", "geort_virtual_base_joint")
    virtual_joint.set("type", "fixed")
    ET.SubElement(virtual_joint, "origin", xyz="0 0 0", rpy=rotation_rpy)
    parent_el = ET.SubElement(virtual_joint, "parent")
    parent_el.set("link", virtual_name)
    child_el = ET.SubElement(virtual_joint, "child")
    child_el.set("link", original_base)
    
    logger.info(f"Added virtual base_link '{virtual_name}' with rpy={rotation_rpy}")
    return virtual_name


def validate_with_sapien(urdf_path: str) -> Tuple[bool, str]:
    """
    Try to load the URDF with SAPIEN to verify compatibility.
    
    Returns (success, message).
    """
    try:
        import sapien.core as sapien
    except ImportError:
        return None, "sapien not installed - skipping load test"
    
    try:
        engine = sapien.Engine()
        scene_config = sapien.SceneConfig()
        scene = engine.create_scene(scene_config)
        
        loader = scene.create_urdf_loader()
        hand = loader.load(urdf_path)
        
        if hand is None:
            return False, "loader.load() returned None"
        
        # Verify we can create pinocchio model (used by GeoRT)
        pmodel = hand.create_pinocchio_model()
        
        active_joints = hand.get_active_joints()
        all_links = hand.get_links()
        
        info = (f"Loaded OK: {len(active_joints)} active joints, "
                f"{len(all_links)} links, pinocchio model OK")
        
        # Verify joint count matches expected 12 DOF
        if len(active_joints) != 12:
            return False, f"{info} - WARNING: expected 12 active joints, got {len(active_joints)}"
        
        return True, info
        
    except Exception as e:
        return False, f"SAPIEN load failed: {e}"


def copy_mesh_files(mesh_src_dir: str, output_mesh_dir: str, dry_run: bool = False) -> int:
    """Copy mesh files from source to output directory."""
    if not os.path.exists(mesh_src_dir):
        logger.error(f"Mesh source directory not found: {mesh_src_dir}")
        return 0
    
    os.makedirs(output_mesh_dir, exist_ok=True)
    
    count = 0
    for f in os.listdir(mesh_src_dir):
        if f.lower().endswith(('.stl', '.obj', '.dae', '.ply')):
            src = os.path.join(mesh_src_dir, f)
            dst = os.path.join(output_mesh_dir, f)
            if not dry_run:
                shutil.copy2(src, dst)
            count += 1
            logger.debug(f"  {'Would copy' if dry_run else 'Copied'}: {f}")
    
    return count


def copy_to_geort_assets(output_dir: str, geort_assets_dir: str, 
                         dry_run: bool = False) -> bool:
    """Copy the processed URDF + meshes to GeoRT's assets directory."""
    if geort_assets_dir is None:
        return False
    
    if dry_run:
        logger.info(f"[DRY RUN] Would copy {output_dir} -> {geort_assets_dir}")
        return True
    
    try:
        if os.path.exists(geort_assets_dir):
            shutil.rmtree(geort_assets_dir)
        shutil.copytree(output_dir, geort_assets_dir)
        logger.info(f"Copied assets to GeoRT: {geort_assets_dir}")
        return True
    except Exception as e:
        logger.error(f"Failed to copy to GeoRT assets: {e}")
        return False


def process_urdf(input_path: str, 
                 output_dir: str,
                 mesh_src_dir: Optional[str] = None,
                 geort_assets_dir: Optional[str] = None,
                 output_filename: str = "xhand_right_nocol.urdf",
                 dry_run: bool = False,
                 validate_only: bool = False,
                 keep_collisions: bool = False,
                 add_virtual_base: bool = False,
                 virtual_base_rpy: str = "0 0 0") -> URDFProcessingReport:
    """
    Main processing pipeline.
    
    Args:
        input_path: Path to the original XHand URDF
        output_dir: Output directory for processed URDF + meshes
        mesh_src_dir: Source directory for mesh files (optional, auto-detected)
        geort_assets_dir: GeoRT assets directory to copy to (optional)
        output_filename: Name for the output URDF file
        dry_run: If True, don't write anything
        validate_only: If True, only validate existing URDF
        keep_collisions: If True, don't remove collision elements
        add_virtual_base: Force adding a virtual base link
        virtual_base_rpy: RPY for virtual base link alignment
    
    Returns:
        URDFProcessingReport with all results
    """
    report = URDFProcessingReport()
    report.input_path = input_path
    report.output_path = os.path.join(output_dir, output_filename)
    
    # ---- Parse URDF ----
    logger.info(f"Parsing URDF: {input_path}")
    try:
        tree = parse_urdf(input_path)
    except Exception as e:
        report.errors.append(f"Failed to parse URDF: {e}")
        return report
    
    root = tree.getroot()
    counts = count_elements(root)
    report.original_link_count = counts["links"]
    report.original_joint_count = counts["joints"]
    report.revolute_joint_count = counts["revolute"]
    report.fixed_joint_count = counts["fixed"]
    
    logger.info(f"  Links: {counts['links']}, Joints: {counts['joints']} "
                f"(revolute: {counts['revolute']}, fixed: {counts['fixed']})")
    logger.info(f"  Collisions: {counts['collisions']}, Visuals: {counts['visuals']}")
    
    # ---- Get base link ----
    base_link_el = root.findall("link")
    if base_link_el:
        report.base_link_name = base_link_el[0].get("name")
        logger.info(f"  Base link: {report.base_link_name}")
    
    if validate_only:
        logger.info("Validate-only mode: skipping modifications")
    else:
        # ---- Step 1: Remove collisions ----
        if not keep_collisions:
            logger.info("Step 1: Removing collision elements...")
            report.collisions_removed = remove_collisions(root)
            logger.info(f"  Removed {report.collisions_removed} collision elements")
        else:
            logger.info("Step 1: Keeping collision elements (--keep-collisions)")
        
        # ---- Step 2: Fix mesh paths ----
        logger.info("Step 2: Fixing mesh paths...")
        report.mesh_paths_fixed = fix_mesh_paths(root)
        logger.info(f"  Fixed {report.mesh_paths_fixed} mesh paths")
        
        # ---- Step 3: Add virtual base if needed ----
        if add_virtual_base:
            logger.info("Step 3: Adding virtual base_link...")
            new_base = add_virtual_base_link(root, report.base_link_name, virtual_base_rpy)
            report.base_link_name = new_base
            report.virtual_base_added = True
        else:
            logger.info("Step 3: Virtual base_link not needed")
    
    # ---- Step 4: Check coordinate system ----
    logger.info("Step 4: Checking coordinate system...")
    coord_ok, coord_notes = check_coordinate_system(root)
    report.coordinate_system_ok = coord_ok
    report.coordinate_system_notes = coord_notes
    if coord_ok:
        logger.info(f"  Coordinate system: OK - matches GeoRT convention")
    else:
        logger.warning(f"  Coordinate system: MISMATCH - {coord_notes}")
        if not add_virtual_base and not validate_only:
            report.warnings.append(
                "Base link coordinate system doesn't match GeoRT convention. "
                "Consider using --add-virtual-base with appropriate --virtual-base-rpy"
            )
    
    # ---- Step 5: Verify mesh files ----
    mesh_filenames = get_mesh_filenames(root)
    
    # Determine the directory to check meshes against
    if validate_only:
        check_dir = os.path.dirname(os.path.abspath(input_path))
    else:
        check_dir = output_dir
    
    # Auto-detect mesh source
    if mesh_src_dir is None:
        input_dir = os.path.dirname(os.path.abspath(input_path))
        candidate = os.path.join(input_dir, "..", "meshes")
        if os.path.exists(candidate):
            mesh_src_dir = candidate
            logger.info(f"  Auto-detected mesh source: {mesh_src_dir}")
        else:
            candidate2 = os.path.join(input_dir, "meshes")
            if os.path.exists(candidate2):
                mesh_src_dir = candidate2
    
    # Copy meshes if processing (not validate-only)
    if not validate_only and mesh_src_dir:
        output_mesh_dir = os.path.join(output_dir, "meshes")
        logger.info(f"Step 5: Copying mesh files from {mesh_src_dir}...")
        copied = copy_mesh_files(mesh_src_dir, output_mesh_dir, dry_run=dry_run)
        logger.info(f"  Copied {copied} mesh files")
    
    # Verify meshes exist
    found, total, missing = verify_mesh_files(mesh_filenames, check_dir)
    report.mesh_files_total = total
    report.mesh_files_found = found
    report.mesh_files_missing = missing
    
    if missing:
        for m in missing:
            report.warnings.append(f"Missing mesh: {m}")
        logger.warning(f"  {len(missing)} mesh files missing!")
    else:
        logger.info(f"  All {total} mesh files verified ✓")
    
    # ---- Step 6: Write output ----
    if not validate_only and not dry_run:
        os.makedirs(output_dir, exist_ok=True)
        output_path = os.path.join(output_dir, output_filename)
        ET.indent(tree, space="  ")
        tree.write(output_path, xml_declaration=True, encoding="utf-8")
        logger.info(f"Step 6: Written URDF to {output_path}")
    elif dry_run:
        logger.info(f"Step 6: [DRY RUN] Would write to {report.output_path}")
    
    # ---- Step 7: Copy to GeoRT assets ----
    if geort_assets_dir and not validate_only:
        logger.info("Step 7: Copying to GeoRT assets...")
        copy_to_geort_assets(output_dir, geort_assets_dir, dry_run=dry_run)
    
    # ---- Step 8: SAPIEN validation ----
    logger.info("Step 8: SAPIEN validation...")
    urdf_to_validate = report.output_path if not validate_only else input_path
    if os.path.exists(urdf_to_validate):
        result, notes = validate_with_sapien(os.path.abspath(urdf_to_validate))
        report.sapien_compatible = result
        report.sapien_notes = notes
    else:
        report.sapien_notes = f"File not found: {urdf_to_validate}"
    
    return report


def generate_geort_config(root: ET.Element, config_output_path: str,
                          urdf_relative_path: str, dry_run: bool = False):
    """
    Generate a GeoRT-compatible config JSON for XHand.
    
    This examines the URDF and auto-generates the configuration.
    """
    # Find base link
    base_link = root.findall("link")[0].get("name")
    
    # Find all revolute joints
    revolute_joints = []
    for joint in root.findall("joint"):
        if joint.get("type") == "revolute":
            revolute_joints.append(joint.get("name"))
    
    # Find tip links (links whose names contain 'tip')
    tip_links = []
    for link in root.findall("link"):
        if "tip" in link.get("name", "").lower():
            tip_links.append(link.get("name"))
    
    # Build parent->child chain mapping
    children_map = {}  # parent_link -> [(child_link, joint_name, joint_type)]
    joint_parent_map = {}  # joint_name -> parent_link
    for joint in root.findall("joint"):
        p = joint.find("parent").get("link")
        c = joint.find("child").get("link")
        jn = joint.get("name")
        jt = joint.get("type")
        if p not in children_map:
            children_map[p] = []
        children_map[p].append((c, jn, jt))
        joint_parent_map[jn] = p
    
    # For each tip, trace back to base to find driving revolute joints
    from collections import deque
    
    # Build reverse map: child_link -> (parent_link, joint_name, joint_type)
    child_to_parent = {}
    for joint in root.findall("joint"):
        p = joint.find("parent").get("link")
        c = joint.find("child").get("link")
        child_to_parent[c] = (p, joint.get("name"), joint.get("type"))
    
    # Human hand keypoint IDs (MediaPipe convention)
    finger_tip_ids = {
        "thumb": 4,
        "index": 8,
        "middle": 12, "mid": 12,
        "ring": 16,
        "pinky": 20,
    }
    
    fingertip_config = []
    for tip_link in tip_links:
        # Trace from tip to base, collecting revolute joints
        chain_joints = []
        current = tip_link
        while current != base_link and current in child_to_parent:
            parent, jname, jtype = child_to_parent[current]
            if jtype == "revolute":
                chain_joints.insert(0, jname)
            current = parent
        
        # Determine finger name
        finger_name = "unknown"
        tip_lower = tip_link.lower()
        for fname in ["thumb", "index", "mid", "ring", "pinky"]:
            if fname in tip_lower:
                finger_name = fname
                break
        
        # Get human hand ID
        human_id = finger_tip_ids.get(finger_name, -1)
        
        # Determine center_offset based on finger direction
        # For XHand: thumb tip extends in Y, others extend in Z
        if finger_name == "thumb":
            center_offset = [0.0, 0.02, 0.0]
        else:
            center_offset = [0.0, 0.0, 0.02]
        
        fingertip_config.append({
            "name": finger_name if finger_name != "mid" else "middle",
            "link": tip_link,
            "joint": chain_joints,
            "center_offset": center_offset,
            "human_hand_id": human_id,
        })
    
    # Sort by human_hand_id
    fingertip_config.sort(key=lambda x: x["human_hand_id"])
    
    config = {
        "name": "xhand_right",
        "urdf_path": urdf_relative_path,
        "base_link": base_link,
        "joint_order": revolute_joints,
        "fingertip_link": fingertip_config,
    }
    
    if not dry_run and config_output_path:
        os.makedirs(os.path.dirname(config_output_path), exist_ok=True)
        with open(config_output_path, 'w') as f:
            json.dump(config, f, indent=4)
        logger.info(f"Generated GeoRT config: {config_output_path}")
    
    return config


def main():
    parser = argparse.ArgumentParser(
        description="Prepare XHand URDF for SAPIEN + GeoRT",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Full processing:
  python3 prepare_xhand_urdf.py \\
    --input  "XHAND1_URDF_ver 1.3/xhand1_right/urdf/xhand_right.urdf" \\
    --output-dir "manus_xhand_teleop/assets/xhand_right/" \\
    --mesh-src "XHAND1_URDF_ver 1.3/xhand1_right/meshes/" \\
    --geort-assets "GeoRT/assets/xhand_right/"

  # Validate existing:
  python3 prepare_xhand_urdf.py --validate-only \\
    --input "manus_xhand_teleop/assets/xhand_right/xhand_right_nocol.urdf"

  # Generate GeoRT config:
  python3 prepare_xhand_urdf.py --gen-config \\
    --input "manus_xhand_teleop/assets/xhand_right/xhand_right_nocol.urdf" \\
    --config-output "manus_xhand_teleop/config/xhand_right.json"
        """
    )
    
    parser.add_argument("--input", "-i", required=True,
                        help="Input URDF file path")
    parser.add_argument("--output-dir", "-o", default=None,
                        help="Output directory for processed URDF + meshes")
    parser.add_argument("--output-filename", default="xhand_right_nocol.urdf",
                        help="Output URDF filename (default: xhand_right_nocol.urdf)")
    parser.add_argument("--mesh-src", default=None,
                        help="Source directory for mesh files (auto-detected if omitted)")
    parser.add_argument("--geort-assets", default=None,
                        help="GeoRT assets directory to copy processed files to")
    parser.add_argument("--validate-only", action="store_true",
                        help="Only validate, don't process")
    parser.add_argument("--dry-run", action="store_true",
                        help="Show what would be done without writing")
    parser.add_argument("--keep-collisions", action="store_true",
                        help="Keep collision elements (default: remove them)")
    parser.add_argument("--add-virtual-base", action="store_true",
                        help="Add a virtual base_link for coordinate alignment")
    parser.add_argument("--virtual-base-rpy", default="0 0 0",
                        help="RPY rotation for virtual base joint (default: '0 0 0')")
    parser.add_argument("--gen-config", action="store_true",
                        help="Generate GeoRT config JSON")
    parser.add_argument("--config-output", default=None,
                        help="Path for generated GeoRT config JSON")
    parser.add_argument("--verbose", "-v", action="store_true",
                        help="Verbose output")
    
    args = parser.parse_args()
    
    if args.verbose:
        logging.getLogger().setLevel(logging.DEBUG)
    
    # Default output directory
    if args.output_dir is None and not args.validate_only:
        args.output_dir = os.path.join(os.path.dirname(args.input), "..", "processed")
        logger.info(f"Default output dir: {args.output_dir}")
    
    if args.validate_only:
        args.output_dir = os.path.dirname(os.path.abspath(args.input))
    
    # Process
    report = process_urdf(
        input_path=args.input,
        output_dir=args.output_dir,
        mesh_src_dir=args.mesh_src,
        geort_assets_dir=args.geort_assets,
        output_filename=args.output_filename,
        dry_run=args.dry_run,
        validate_only=args.validate_only,
        keep_collisions=args.keep_collisions,
        add_virtual_base=args.add_virtual_base,
        virtual_base_rpy=args.virtual_base_rpy,
    )
    
    # Generate config if requested
    if args.gen_config:
        tree = parse_urdf(args.input)
        root = tree.getroot()
        config_out = args.config_output
        if config_out is None:
            config_out = os.path.join(args.output_dir or ".", "config", "xhand_right.json")
        
        urdf_rel = f"./assets/xhand_right/{args.output_filename}"
        config = generate_geort_config(root, config_out, urdf_rel, dry_run=args.dry_run)
        
        if args.dry_run:
            print("\n[DRY RUN] Generated config:")
            print(json.dumps(config, indent=2))
    
    # Print report
    report.print_report()
    
    # Exit code
    if report.errors:
        sys.exit(1)
    elif report.warnings:
        sys.exit(0)
    else:
        sys.exit(0)


if __name__ == "__main__":
    main()
