from enum import Enum

# This class defines the poses that are reachable for each device in the production line.
# Note that the numbering of the devices for the production line starts from
# the left and goes to the right; sensors and the camera are excluded from this count.
# This class is used for checking or searching if a device can reach a pose and execute
# an operation.
# For each device, it is indicated its type, the initial_poses, the final_poses, and the 
# near_case_poses for the robot only.
# - initial_poses: For robots, these are divided into case_poses and element_poses.
#                  These poses indicate the starting position (or picking for the robot)
#                  of a product (or even an element for the robot) when that particular device
#                  starts its operations.
# - final_poses: For all devices, these are the poses where they can carry (or place for a robot)
#                the product at the end of their operations.
# - near_case_poses: These poses exist only for robots and are the poses needed for inserting 
#                    or removing an element.
# For better understanding of the pose nomenclature, refer to the "Pose" file
# in the "production_line_device" package.
class DevicePoses(Enum):
    device_1 = { 
        "type": "robot",
        "initial_poses": {
            "case_poses": ["Pose_C_W1_1", "Pose_C_W1_2", "Pose_C_W1_3"],
            "element_poses": ["Pose_E_W1_R_1", "Pose_E_W1_G_1", "Pose_E_W1_B_1"]
        },
        "near_case_poses": ["Pose_E_Top_1_2"],
        "final_poses": ["Pose_C_1_2"]
    }
    device_2 = { 
        "type": "conveyor belt",
        "initial_poses": ["Pose_C_1_2"],
        "final_poses": ["Pose_C_2_3"]
    }
    device_3 = {
        "type": "robot",
        "initial_poses": {
            "case_poses": ["Pose_C_2_3"],
            "element_poses": ["Pose_E_W2_R_1", "Pose_E_W2_G_1", "Pose_E_W2_B_1",
                              "Pose_E_W2_R_2", "Pose_E_W2_G_2", "Pose_E_W2_B_2",
                              "Pose_E_W2_R_3", "Pose_E_W2_G_3", "Pose_E_W2_B_3",
                              "Pose_E_W2_R_4", "Pose_E_W2_G_4", "Pose_E_W2_B_4"]
        },
        "near_case_poses": ["Pose_E_Top_2_3", "Pose_E_Top_3_4", "Pose_E_Top_4_5",
                            "Pose_E1_3_4", "Pose_E2_3_4", "Pose_E3_3_4",
                            "Pose_E1_4_5", "Pose_E2_4_5", "Pose_E3_4_5"],
        "final_poses": ["Pose_C_3_4", "Pose_C_Help_3_4", "Pose_C_Help_4_5",
                        "Pose_C_W3_1", "Pose_C_W3_2", "Pose_C_W3_3"]
    }
    device_4 = {
        "type": "Rotating Table",
        "initial_poses": ["Pose_C_3_4"],
        "final_poses": ["Pose_C_4_5"]
    }
    device_5 = {
        "type": "robot",
        "initial_poses": {
            "case_poses": ["Pose_C_4_5", "Pose_C_3_4"],
            "element_poses": ["Pose_E_W2_R_1", "Pose_E_W2_G_1", "Pose_E_W2_B_1",
                              "Pose_E_W2_R_2", "Pose_E_W2_G_2", "Pose_E_W2_B_2",
                              "Pose_E_W2_R_3", "Pose_E_W2_G_3", "Pose_E_W2_B_3",
                              "Pose_E_W2_R_4", "Pose_E_W2_G_4", "Pose_E_W2_B_4"]
        },
        "near_case_poses": ["Pose_E_Top_3_4", "Pose_E_Top_4_5", "Pose_E_Top_3_4_Rotated",
                            "Pose_E_Top_4_5_Rotated", "Pose_E1_3_4", "Pose_E2_3_4",
                            "Pose_E3_3_4", "Pose_E1_4_5","Pose_E2_4_5", "Pose_E3_4_5"],
        "final_poses": ["Pose_C_W3_1", "Pose_C_W3_2", "Pose_C_W3_3"]
    }