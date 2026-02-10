import yaml
import os
from ament_index_python.packages import get_package_share_directory

class CamTable:

    CAMTABLE_PATH = os.path.join(
        get_package_share_directory("redshift_odometry"),
        "config",
        "camtable.yaml"
    )

    # this will be populated from YAML
    cam_table = []

    @staticmethod
    def load_cam_table():
        print(f"Camera table loaded from {CamTable.CAMTABLE_PATH}")

        if not os.path.exists(CamTable.CAMTABLE_PATH):
            print(f"ERROR: Camera table YAML not found at {CamTable.CAMTABLE_PATH}")
            raise FileNotFoundError(
                f"Camera table YAML not found at {CamTable.CAMTABLE_PATH}"
            )

        with open(CamTable.CAMTABLE_PATH, "r") as f:
            data = yaml.safe_load(f)

        if data is None or "cameras" not in data:
            print("ERROR: Invalid camera table YAML: missing 'cameras' key")
            raise ValueError(
                "Invalid camera table YAML: missing 'cameras' key"
            )

        CamTable.cam_table = data["cameras"]

        profile = data.get("profile", "unknown")
        print(f"Loaded camera profile '{profile}' with {len(CamTable.cam_table)} cameras")

    @staticmethod
    def compound_quat(entry):
        # following quaternion is the adjustment of the camera from FLU to RDF
        x2, y2, z2, w2 = -0.5, 0.5, -0.5, 0.5

        x1 = entry["qx"]
        y1 = entry["qy"]
        z1 = entry["qz"]
        w1 = entry["qw"]

        w = w2 * w1 - x2 * x1 - y2 * y1 - z2 * z1
        x = w2 * x1 + x2 * w1 + y2 * z1 - z2 * y1
        y = w2 * y1 - x2 * z1 + y2 * w1 + z2 * x1
        z = w2 * z1 + x2 * y1 - y2 * x1 + z2 * w1

        return w, x, y, z


# load the table at import time (same behavior as before)
CamTable.load_cam_table()

