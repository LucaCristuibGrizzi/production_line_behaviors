#!/usr/bin/env python
import rospkg
import sys
from flexbe_core import EventState

# Append Custom Path to Execution
package_path = rospkg.RosPack().get_path('production_line_flexbe_states')
sys.path.append(f'{package_path}/src/production_line_flexbe_states/')

# Custom Imports
from Utils.Warehouses import Warehouses

class MainState(EventState):
    '''
    Define the pick-and-place positions for robots operations to assemble the product based on the color code.

    ># robots_operations   dictionary   Dictionary of robot operations.
    ># color_code          list         Color code of the product to be assembled by the production line.

    #>  robots_operations  dictionary   Dictionary of robot operations.
    #>  warehouses         dictionary   Dictionary of the state of the warehouses.

    <= done                Done setting the robots operations.
    '''

    def __init__(self):
        # Declare outcomes, input_keys and output_keys.
        super(MainState, self).__init__(outcomes=['done'], 
                                        input_keys=['robots_operations', 'color_code'],
                                        output_keys=['robots_operations', 'warehouses'])
        
        # Initialize state parameters for picking positions.
        self._pick_E1 = None
        self._pick_Case_W1 = None
        self._pick_E2 = None
        self._pick_E3 = None
        
    def execute(self, userdata):
        # The robots operations are already set when the state is active, so
        # it can return the 'done' outcome.
        return 'done'
    
    def on_enter(self, userdata):
        # When the state becomes active, initialize the warehouses status from the Warehouses module.
        # This status represents the initial condition of the warehouses.
        userdata.warehouses = {warehouse.name: warehouse.value for warehouse in Warehouses}

        # Set the state parameters for picking positions based on the product assembly requirements.
        self._selectRobotPoses(userdata)

        # Set the dictionary for robots operations.
        # Note:
        # - Each key in the dictionary corresponds to a robot.
        # - If a robot's operations are consecutive, they can be grouped in a 
        #   single element in the list associated with the robot.
        # - Each operation consists of a pick-and-place, with the order always 
        #   being [pickPose, placePose].
        # - This dictionary defines the positions for assembling a single product.
        userdata.robots_operations = {
            "robot_1": [{
                "n_operations": 2,
                "operation_1": [ self._pick_Case_W1, "Pose_C_1_2"],
                "operation_2": [ self._pick_E1, "Pose_E_Top_1_2"]
            }],

            "robot_2": [{
                "n_operations": 2,
                "operation_1": [ self._pick_E2, "Pose_E_Top_2_3"],
                "operation_2": [ "Pose_C_2_3", "Pose_C_3_4"]
            }],

            "robot_3": [{
                "n_operations": 2,
                "operation_1": [ self._pick_E3, "Pose_E_Top_4_5"],
                "operation_2": [ "Pose_C_4_5", "Pose_C_4_5_View"]
            },
            {
                "n_operations": 1,
                "operation_1": [ "Pose_C_4_5_View", "Pose_C_W3_1"],
            }]
        }

    # Method to select robot poses that are not fixed for every production cycle.
    def _selectRobotPoses(self, userdata):
        self._pick_E1 = self._selectWarehousePose(userdata,"warehouse_1", "E", colorObject=userdata.color_code[0])
        self._pick_Case_W1 = self._selectWarehousePose(userdata, "warehouse_1", "C")
        self._pick_E2 = self._selectWarehousePose(userdata, "warehouse_2", "E", colorObject=userdata.color_code[1])
        self._pick_E3 = self._selectWarehousePose(userdata, "warehouse_2", "E", colorObject=userdata.color_code[2])


    # Method to select the warehouse position where the robot can find the required element based on the warehouse status.
    # The warehouse, object type ("C" for case and "E" for elements), and color (if needed) must be specified.
    def _selectWarehousePose(self, userdata, warehouse, objectType, colorObject = ""):
        warehousePoses = userdata.warehouses[warehouse]
        warehouseNumber = warehouse[-1]
        subPoseName = "Pose_%s_W%s_%s" % (objectType, warehouseNumber, colorObject)

        # Search for a pose in the warehouse that meets the requirements and is not void.
        for poseName, state in warehousePoses.items():
            if (subPoseName in poseName and state):
                userdata.warehouses[warehouse][poseName] = False
                return poseName