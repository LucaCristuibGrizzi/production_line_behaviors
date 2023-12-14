from enum import Enum

# This class represents the initial state of the warehouses
# (or the supposed one).
# True indicates that the pose is occupied by an object.
# False indicates that the pose is empty.
# For better understanding of the pose nomenclature, refer to the "Pose" file
# in the "production_line_device" package.
class Warehouses(Enum):
    warehouse_1 = {
        "Pose_C_W1_1": True,
        "Pose_C_W1_2": True,
        "Pose_C_W1_3": True,
        "Pose_E_W1_R_1": True,
        "Pose_E_W1_G_1": True,
        "Pose_E_W1_B_1": True
    }

    warehouse_2 = {
        "Pose_E_W2_R_1": True,
        "Pose_E_W2_G_1": True,
        "Pose_E_W2_B_1": True,
        "Pose_E_W2_R_2": True,
        "Pose_E_W2_G_2": True,
        "Pose_E_W2_B_2": True,
        "Pose_E_W2_R_3": False,
        "Pose_E_W2_G_3": True,
        "Pose_E_W2_B_3": False,
        "Pose_E_W2_R_4": False,
        "Pose_E_W2_G_4": False,
        "Pose_E_W2_B_4": False
    }

    warehouse_3 = {
        "Pose_C_W3_1": False,
        "Pose_C_W3_2": False,
        "Pose_C_W3_3": False
    }