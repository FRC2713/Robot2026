#!/usr/bin/env python3
"""
(UNTESTED) -- not sure if needed but i couldnt find a place in the web ui to import a calibration, so this is an attempt to save a calibration to the sqlite file on the jetson

Import a PhotonCal calibration JSON into PhotonVision's SQLite database.
Run this on the Jetson after copying both files there.

Usage: python3 import_calibration.py <calibration.json> <camera_name>
Example: python3 import_calibration.py Camera_1_1280x720.json spare_thrifty
"""

import json
import sqlite3
import sys
from pathlib import Path

PHOTON_DB = "/opt/photonvision/photonvision_config/photon.sqlite"


def load_mrcal_calibration(cal_path: str) -> dict:
    """Load a PhotonCal/mrcal format calibration JSON."""
    with open(cal_path) as f:
        cal = json.load(f)

    # Convert to PhotonVision's CameraCalibrationCoefficients format
    return {
        "resolution": cal["resolution"],
        "cameraIntrinsics": cal["cameraIntrinsics"],
        "distCoeffs": cal["distCoeffs"],
        "calobjectWarp": cal.get("calobjectWarp", [0.0, 0.0]),
        "calobjectSize": cal.get("calobjectSize", {"width": 12, "height": 9}),
        "calobjectSpacing": cal.get("calobjectSpacing", 0.03),
        "lensmodel": cal.get("lensmodel", "LENSMODEL_OPENCV"),
        "observations": [],  # Empty - we don't have the original snapshots
    }


def import_calibration(cal_path: str, camera_name: str | None):
    """Import calibration into the camera's config in the SQLite database."""
    # Load and convert calibration
    pv_cal = load_mrcal_calibration(cal_path)
    resolution = pv_cal["resolution"]
    print(f"Loaded calibration for {resolution['width']}x{resolution['height']}")

    # Connect to PhotonVision's database
    conn = sqlite3.connect(PHOTON_DB)
    cursor = conn.cursor()

    # Get current camera config
    if camera_name is None:
        # Get all cameras and use the first/only one
        cursor.execute("SELECT unique_name, config_json FROM cameras")
        rows = cursor.fetchall()
        if len(rows) == 0:
            print("Error: No cameras found in database.")
            conn.close()
            sys.exit(1)
        if len(rows) > 1:
            print(f"Multiple cameras found. Please specify one:")
            for r in rows:
                cfg = json.loads(r[1])
                print(f"  {r[0]} (nickname: {cfg.get('nickname', 'unknown')})")
            conn.close()
            sys.exit(1)
        camera_name = rows[0][0]
        row = (rows[0][1],)
        print(f"Using camera: {camera_name}")
    else:
        cursor.execute("SELECT config_json FROM cameras WHERE unique_name = ?", (camera_name,))
        row = cursor.fetchone()

        if row is None:
            # List available cameras
            cursor.execute("SELECT unique_name, config_json FROM cameras")
            rows = cursor.fetchall()
            print(f"Error: Camera '{camera_name}' not found in database.")
            print("Available cameras:")
            for r in rows:
                cfg = json.loads(r[1])
                print(f"  {r[0]} (nickname: {cfg.get('nickname', 'unknown')})")
            conn.close()
            sys.exit(1)

    config = json.loads(row[0])
    print(f"Found camera config for '{camera_name}'")

    # Add or replace calibration for this resolution
    calibrations = config.get("calibrations", [])

    # Remove existing calibration at same resolution
    calibrations = [
        c for c in calibrations
        if c.get("resolution") != resolution
    ]

    # Add new calibration
    calibrations.append(pv_cal)
    config["calibrations"] = calibrations

    # Write back to database
    cursor.execute(
        "UPDATE cameras SET config_json = ? WHERE unique_name = ?",
        (json.dumps(config), camera_name)
    )
    conn.commit()
    conn.close()

    print(f"Successfully imported calibration into '{camera_name}'")
    print("Restart PhotonVision for changes to take effect:")
    print("  sudo systemctl restart photonvision")


if __name__ == "__main__":
    if len(sys.argv) == 2:
        # Just calibration file - use the only camera in the DB
        import_calibration(sys.argv[1], None)
    elif len(sys.argv) == 3:
        import_calibration(sys.argv[1], sys.argv[2])
    else:
        print(f"Usage: {sys.argv[0]} <calibration.json> [camera_name]")
        print(f"Example: {sys.argv[0]} Camera_1_1280x720.json")
        print(f"         {sys.argv[0]} Camera_1_1280x720.json spare_thrifty")
        sys.exit(1)
