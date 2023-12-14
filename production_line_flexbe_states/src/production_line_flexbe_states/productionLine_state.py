#!/usr/bin/env python
import rospy, rostopic, rospkg
import sys
import copy
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyPublisher, ProxySubscriberCached

# Append Custom Path to Execution
package_path = rospkg.RosPack().get_path('production_line_flexbe_states')
sys.path.append(f'{package_path}/src/production_line_flexbe_states/')

# Custom Imports
from Utils.Warehouses import Warehouses
from production_line_device.msg import InitialCondition

class ProductionLineState(EventState):
    '''
    Define the robot poses needed to correctly assemble the three products and 
    initiate the working of the production line. Additionally, manage the state of the 
    warehouses and receive error notifications from the production line.

    -- topicPub             string      The publisher topic for communicating with the production line node.
    -- topicSub             string      The subscriber topic for communicating with the production line node.
    -- clear                bool        Drop the last message on this topic upon entering to only 
                                        handle messages received since this state is active.
    
    ># first_color_code     list        Color code of the first product to be realized.
    ># second_color_code    list        Color code of the second product to be realized.
    ># third_color_code     list        Color code of the third product to be realized.
    ># warehouses           dictionary  Dictionary of the state of the warehouses.

    #> warehouses           dictionary  Dictionary of the state of the warehouses.
    #> error_type           string      The error type detected.
    #> speed                int         Percentage of speed communicated by the fault device.
    #> faulty_device        int         The number of the faulty device.
    #> n_element            int         The number of elements that should be inserted in the case at the work
                                        station where the assembly error is detected.
    #> color_code_detected  list        The color code detected by the camera.
    #> name_warehouse       string      The name of the warehouse where the solution for the assembly error must be searched.
    #> color_code           list        The color code required to realize the product for which the assembly error was detected.

    <= done         All the operations of the production line are completed.
    <= failed       Failed operation (Subscriber connection failure).
    <= error        Error in production line operations.
    '''

    def __init__(self, topicPub, topicSub, clear = False):
        # Declare outcomes, input_keys and output_keys.
        super(ProductionLineState, self).__init__(outcomes = ['done', 'failed', 'error'],
                                                  input_keys = ['first_color_code', 'second_color_code',
                                                                'third_color_code', 'warehouses'],
                                                  output_keys = ['warehouses', 'error_type', 'speed', 'faulty_device',
                                                                 'n_element', 'color_code_detected',
                                                                 'name_warehouse', 'color_code'])
        
        # Initialize state parameters for the publisher and create the publisher.
        self._topicPub = topicPub
        self._pub = ProxyPublisher({self._topicPub: InitialCondition})

        # Initialize state parameters for later use in the subscriber.
        self._topicSub = topicSub
        self._connected = False
        self._clear = clear

        # Poses to define based on the color code to assembly the first product.
        self._pick_E1_CC1 = None
        self._pick_Case_W1_CC1 = None
        self._pick_E2_CC1 = None
        self._pick_E3_CC1 = None
        self._place_Case_W3_CC1 = None

        # Poses to define based on the color code to assembly the second product.
        self._pick_E1_CC2 = None
        self._pick_Case_W1_CC2 = None
        self._pick_E2_CC2 = None
        self._pick_E3_CC2 = None
        self._place_Case_W3_CC2 = None

        # Poses to define based on the color code to assembly the third product.
        self._pick_E1_CC3 = None
        self._pick_Case_W1_CC3 = None
        self._pick_E2_CC3 = None
        self._pick_E3_CC3 = None
        self._place_Case_W3_CC3 = None

        # Initialize the state parameter to determine if setting the position or not.
        self._setPoses = True

        # Attempt to subscribe to the topic and check if the connection was successful.
        if not self._connect():
            Logger.logwarn('Topic %s for state %s not yet available.\n'
                           'Will try again when entering the state...' % (self._topicSub, self.name))

    def execute(self, userdata):
        # Check if the subscribe was unsuccessful.
        if not self._connected:
            return 'failed'

        # Check if the received message indicates the completion of the operations of the production line,
        # to update the status of the warehouse or if an error was detected in the production line.
        if self._sub.has_msg(self._topicSub):
            status = self._sub.get_last_msg(self._topicSub)

            # If all three products are assembled correctly, then return 'done'.
            if status.finished:
                return 'done'
            # Management of the error detected.
            elif status.error:
                error_type = status.error_type

                # Based on the error detected, define the value of the userdata needed for 
                # managing the error.
                if error_type == "slowdown":
                    userdata.error_type = error_type
                    userdata.speed = status.speed
                elif error_type == "component_not_arrived":
                    userdata.error_type = error_type
                    userdata.faulty_device = status.faulty_device
                elif error_type == "assembly":
                    userdata.error_type = error_type
                    userdata.faulty_device = status.faulty_device
                    userdata.n_element = status.nElements
                    userdata.color_code_detected = status.colorCodeDetected
                    userdata.name_warehouse = status.warehouseName 
                    nProduct = status.nProduct

                    # Select the right color code for the product for which the assembly error was detected.
                    if nProduct == 1:
                        userdata.color_code = userdata.first_color_code
                    elif nProduct == 2:
                        userdata.color_code = userdata.second_color_code
                    elif nProduct == 3:
                        userdata.color_code = userdata.third_color_code

                return 'error'
            # When a robot of the production line completes successfully an operation, then communicate at this
            # state to update the status of the position of the warehouse where it has removed an element.
            # With this, the state of the warehouses is always updated in real-time and is needed for 
            # searching a solution for an assembly error when it occurs.
            else:
                self._sub.remove_last_msg(self._topicSub)
                self._setStatePoseWarehouse(userdata, status.warehouseName, status.poseWarehouse, False)
                print(userdata.warehouses)
    
    def on_enter(self, userdata):
        # When the state becomes active, set the poses for the robot to execute the correct assembly of the three products.
        # This first check is needed because after managing an error, this state is reactivated.
        # In this case, the poses don't need to be redefined; only the publish is needed to indicate restarting the working.
        if self._setPoses:
            # Define the userdata for the warehouses status with a deepcopy of the module Warehouses.
            # This destroys the link between the value in the module and the value in the userdata.
            userdata.warehouses = copy.deepcopy({warehouse.name: warehouse.value for warehouse in Warehouses})

            self._selectRobotPoses(userdata)

            # The state of the positions of the warehouses was changed during the execution of selectRobotPoses,
            # but for managing their status in real-time, they need to be reset to the real initial condition.
            self._setStatePoseWarehouse(userdata, "warehouse_1", self._pick_E1_CC1, True)
            self._setStatePoseWarehouse(userdata, "warehouse_2", self._pick_E2_CC1, True)
            self._setStatePoseWarehouse(userdata, "warehouse_2", self._pick_E3_CC1, True)
            self._setStatePoseWarehouse(userdata, "warehouse_1", self._pick_E1_CC2, True)
            self._setStatePoseWarehouse(userdata, "warehouse_2", self._pick_E2_CC2, True)
            self._setStatePoseWarehouse(userdata, "warehouse_2", self._pick_E3_CC2, True)
            self._setStatePoseWarehouse(userdata, "warehouse_1", self._pick_E1_CC3, True)
            self._setStatePoseWarehouse(userdata, "warehouse_2", self._pick_E2_CC3, True)
            self._setStatePoseWarehouse(userdata, "warehouse_2", self._pick_E3_CC3, True)

            # This allows defining the position only if the work of the production line is restarted from the beginning.
            self._setPoses = False

        # Set the value of the message InitialCondition to publish at the production line node.
        initialCondition = InitialCondition()
        initialCondition.firstColorCode = userdata.first_color_code
        initialCondition.secondColorCode = userdata.second_color_code
        initialCondition.thirdColorCode = userdata.third_color_code
        initialCondition.robot1Poses = [self._pick_Case_W1_CC1, self._pick_E1_CC1,
                                        self._pick_Case_W1_CC2, self._pick_E1_CC2 ,
                                        self._pick_Case_W1_CC3 , self._pick_E1_CC3]
        initialCondition.robot2Poses = [self._pick_E2_CC1 , self._pick_E2_CC2, self._pick_E2_CC3]
        initialCondition.robot3PosesFirstOperation = [self._pick_E3_CC1 , self._pick_E3_CC2, self._pick_E3_CC3]
        initialCondition.robot3PosesSecondOperation = [self._place_Case_W3_CC1, self._place_Case_W3_CC2, self._place_Case_W3_CC3]

        self._pub.publish(self._topicPub, initialCondition)
            

        # Attempt to subscribe to the topic if the connection was not successful during initialization.
        if not self._connected:
            if self._connect():
                Logger.loginfo('Successfully subscribed to previously unavailable topic %s' % self._topicSub)
            else:
                Logger.logwarn('Topic %s still not available, giving up.' % self._topicSub)

        # Remove the last message on the topic if the connection is successful and clearing is enabled.
        if self._connected and self._clear and self._sub.has_msg(self._topicSub):
            self._sub.remove_last_msg(self._topicSub)

    # Method to search and define all the robot poses needed to assemble three products correctly.
    def _selectRobotPoses(self, userdata):
        # Select Poses for first color code
        self._pick_Case_W1_CC1 = self._selectWarehousePose(userdata, "warehouse_1", "C")
        self._pick_E1_CC1 = self._selectWarehousePose(userdata,"warehouse_1", "E", colorObject=userdata.first_color_code[0]) 
        self._pick_E2_CC1 = self._selectWarehousePose(userdata, "warehouse_2", "E", colorObject=userdata.first_color_code[1])
        self._pick_E3_CC1 = self._selectWarehousePose(userdata, "warehouse_2", "E", colorObject=userdata.first_color_code[2])
        self._place_Case_W3_CC1 = self._selectWarehousePose(userdata, "warehouse_3", "C")

        # Select Poses for second color code
        self._pick_Case_W1_CC2 = self._selectWarehousePose(userdata, "warehouse_1", "C")
        self._pick_E1_CC2 = self._selectWarehousePose(userdata,"warehouse_1", "E", colorObject=userdata.second_color_code[0]) 
        self._pick_E2_CC2 = self._selectWarehousePose(userdata, "warehouse_2", "E", colorObject=userdata.second_color_code[1])
        self._pick_E3_CC2 = self._selectWarehousePose(userdata, "warehouse_2", "E", colorObject=userdata.second_color_code[2])
        self._place_Case_W3_CC2 = self._selectWarehousePose(userdata, "warehouse_3", "C")

        # Select Poses for third color code
        self._pick_Case_W1_CC3 = self._selectWarehousePose(userdata, "warehouse_1", "C")
        self._pick_E1_CC3 = self._selectWarehousePose(userdata,"warehouse_1", "E", colorObject=userdata.third_color_code[0]) 
        self._pick_E2_CC3 = self._selectWarehousePose(userdata, "warehouse_2", "E", colorObject=userdata.third_color_code[1])
        self._pick_E3_CC3 = self._selectWarehousePose(userdata, "warehouse_2", "E", colorObject=userdata.third_color_code[2])
        self._place_Case_W3_CC3 = self._selectWarehousePose(userdata, "warehouse_3", "C")

    # Method to select the warehouse position where the robots can find the required element or place the
    # final product in the warehouse_3 based on the warehouse status.
    # The warehouse, object type ("C" for case and "E" for elements), and color (if needed) must be specified.
    def _selectWarehousePose(self, userdata, warehouse, objectType, colorObject = ""):
        warehousePoses = userdata.warehouses[warehouse]
        warehouseNumber = warehouse[-1]
        subPoseName = "Pose_%s_W%s_%s" % (objectType, warehouseNumber, colorObject)

        # Search for a pose in the warehouse that meets the requirements and is not void for picking an element
        # or is void for placing the final product in warehouse_3.
        for poseName, state in warehousePoses.items():
            if warehouse == "warehouse_3":
                if (subPoseName in poseName and not state):
                    userdata.warehouses[warehouse][poseName] = True
                    return poseName
            else:
                if (subPoseName in poseName and state):
                    userdata.warehouses[warehouse][poseName] = False
                    return poseName
    
    # Method to change te status at the specified position of a warehouse.
    def _setStatePoseWarehouse(self, userdata, warehouse, poseName, state):
        userdata.warehouses[warehouse][poseName] = state
                
                
    # Method that tries to establish a connection to the specified topic.
    # Returns True if the connection is successful, otherwise returns False.
    def _connect(self):
        global_topic = self._topicSub
        if global_topic[0] != '/':
            global_topic = rospy.get_namespace() + global_topic
        msg_type, msg_topic, _ = rostopic.get_topic_class(global_topic)
        if msg_topic == global_topic:
            self._sub = ProxySubscriberCached({self._topicSub: msg_type})
            self._connected = True
            return True
        return False