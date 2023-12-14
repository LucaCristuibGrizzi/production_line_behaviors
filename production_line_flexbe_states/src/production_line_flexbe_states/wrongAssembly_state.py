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
from Utils.DevicePoses import DevicePoses
from production_line_device.msg import WrongAssemblySolution

class WrongAssemblyState(EventState):
    '''
    Manage the error of a product not assembled correctly by the production line.

    -- topicPub             string      The publisher topic for communicating with the devices.
    -- topicSub             string      The subscriber topic for communicating with the devices
    -- clear                bool        Drop the last message on this topic upon entering to only 
                                        handle messages received since this state is active.

    ># faulty_device        int         The number of the faulty device.
    ># n_line_device        int         The number of devices that are present in the production line 
                                        (excluding sensors and cameras).
    ># color_code           list        The color code required to realize the product for which the assembly error was detected.
    ># color_code_detected  list        The color code detected by the camera.
    ># n_element            int         The number of elements that should be inserted in the case at the work station where
                                        the assembly error is detected.
    ># warehouses           dictionary  Dictionary of the state of the warehouses.
    ># name_warehouse       string      The name of the warehouse where the solution for the assembly error must be searched.

    #> warehouses           dictionary  Dictionary of the state of the warehouses.

    <= done                 The corrective operation has been executed.
    <= failed               Failed operation (Subscriber connection failure).
    <= not_found            There is no solution for the error detected.
    '''

    def __init__(self,  topicPub, topicSub, clear = False):
        # Declare outcomes, input_keys and output_keys.
        super(WrongAssemblyState, self).__init__(outcomes=['done', 'failed', 'not_found'],
                                                 input_keys=['faulty_device', 'n_line_device', 
                                                             'color_code', 'color_code_detected',
                                                             'n_element', 'warehouses', 'name_warehouse'],
                                                 output_keys=['warehouses'])
        
        # Initialize state parameters for the publisher and create the publisher.
        self._topicPub = topicPub
        self._pub = ProxyPublisher({self._topicPub: WrongAssemblySolution})

        # Initialize state parameters for later use in the subscriber.
        self._topicSub = topicSub
        self._connected = False
        self._clear = clear

        # Create state parameter for later use in the searching and application of the solution.
        self._elementToRemove = None
        self._nextRobot = None      # Used if the corrective operation is dual robot.
        self._previousRobot = None  # Used if the corrective operation is dual robot.
        self._singleRobot = None    # Used if the corrective operation is single robot.
        self._commands1 = []
        self._commands2 = None
        self._solutionFound = True
        self._doubleRobot = False
        self._warehousesInitialState = None
        self._new_color_code = None

        # Attempt to subscribe to the topic and check if the connection was successful.
        if not self._connect():
            Logger.logwarn('Topic %s for state %s not yet available.\n'
                           'Will try again when entering the state...' % (self._topicSub, self.name))
            
    def execute(self, userdata):
        # Check if the subscribe was unsuccessful.
        if not self._connected:
            return 'failed'
        
        # If a solution for the error detected isn't found, return 'not_found'.
        # The state of the warehouses is restored to its initial condition upon entering this state.
        # This restoration is necessary because, during the procedure of searching for
        # a solution, the state can be modified.
        if not self._solutionFound:
            userdata.warehouses = self._warehousesInitialState
            return 'not_found'
        
        
        if self._doubleRobot:

            # If the corrective operation is dual robots, a procedure is initiated after receiving the message from the first robot.
            # This procedure alternately publishes commands to the second robot and receives messages in the subscriber.
            # This is necessary because the second robot may perform multiple removals and insertions of elements in the case.
            if self._sub.has_msg(self._topicSub) and self._sub.get_last_msg(self._topicSub).data:
                self._sub.remove_last_msg(self._topicSub)
                if self._commands2 != None:
                    self._pub.publish(self._topicPub, self._commands2)
                    self._commands2 = None
                # After the second robot completes all of its operations, the first one needs to execute its final operation.
                # This step is crucial to return the product to the standard pose in the manufacturing process.
                elif len(self._commands1) != 0:
                    self._pub.publish(self._topicPub, self._commands1[0])
                    del self._commands1[0]
                else:
                    return 'done'
        else:
            # If the corrective operation is single robot, check if the message of the complete operation has arrived.
            if self._sub.has_msg(self._topicSub) and self._sub.get_last_msg(self._topicSub).data:
                self._sub.remove_last_msg(self._topicSub)
                return 'done'

    
    def on_enter(self, userdata):
        # Save the initial state of the warehouses. 
        # It is needed for a restoration if a solution can't be found.   
        self._warehousesInitialState = copy.deepcopy(userdata.warehouses)

        # Reset the state parameters indicating the solution status and dual robot operation.
        self._solutionFound = True
        self._doubleRobot = False

        # The color code is respected, but one or more required elements are missing.
        # Recovery for this type of error can be executed using only one robot.
        if userdata.color_code_detected == userdata.color_code[:len(userdata.color_code_detected)]:
            # Define the robot for the operation.
            self._selectRobot(userdata)
            singleRobot = "device_%i" % self._singleRobot

            # Define the pose where the element of the correct color can be picked from the warehouse.
            element_Pick_Pose = self._searchWarehousePose(userdata)

            # Define the pose where the robot will insert the element into the case.
            if userdata.faulty_device != userdata.n_line_device:
                element_Place_Pose = "Pose_E_Top_%i_%i" % (userdata.faulty_device, userdata.faulty_device+1)
            else:
                element_Place_Pose = "Pose_E_Top_%i_%i" % (userdata.faulty_device-1, userdata.faulty_device)

            # Check if the selected robot can reach both poses. If it can, create the command and publish it.
            if (self._checkPose(singleRobot, element_Pick_Pose, "Warehouse") and
                self._checkPose(singleRobot, element_Place_Pose, "Element")):
                command = WrongAssemblySolution()
                command.deviceNumber = self._singleRobot
                command.assistantRobot = False
                command.nOperations = 1
                command.pickPoses = [element_Pick_Pose]
                command.placePoses = [element_Place_Pose]
                self._commands1.append(command)
            else:
                self._solutionFound = False

            if self._solutionFound:
                self._pub.publish(self._topicPub, self._commands1[0])

        # The color code is not respected, so some elements may be removed from the case and others may be inserted.
        # Recovery for this type of error must be executed using two robots (one is the helper, and the other is operative).
        else:
            # Define the elements of the product that need to be removed and select the two robots.
            elementsToRemove = self._findElementToRemove(userdata)
            self._selectRobots(userdata)
            nextRobot = "device_%i" % self._nextRobot
            previousRobot = "device_%i" % self._previousRobot
            self._doubleRobot = True

            # Define the pose where the helper robot needs to place and hold the product to allow removal
            # of the elements by the other robot.
            if userdata.faulty_device != userdata.n_line_device:
                case_Pose = "Pose_C_Help_%i_%i" % (userdata.faulty_device, userdata.faulty_device+1)
            else:
                case_Pose = "Pose_C_Help_%i_%i" % (userdata.faulty_device-1, userdata.faulty_device)

            # Check if the helper robot can reach this pose. If it can, create the command.
            if self._checkPose(previousRobot, case_Pose, "Case"):
                command = WrongAssemblySolution()
                command.deviceNumber = self._previousRobot
                command.assistantRobot = True
                command.helpPose = case_Pose
                self._commands1.append(command)

                # The helper robot maintains the pose of the first command for the entire duration of the operations.
                # So, at the end, it needs a second command that indicates returning home.
                command = WrongAssemblySolution()
                command.deviceNumber = self._previousRobot
                command.assistantRobot = True
                command.goHome = True
                command.helpPose = case_Pose
                self._commands1.append(command)   
            else:
                self._solutionFound = False

            # Define the pick pose and the place pose for the operative robot to remove the element from the case
            # and place it in the correct position in the warehouse.
            pick_Poses = []
            place_Poses = []
            for element in elementsToRemove:
                if userdata.faulty_device != userdata.n_line_device: 
                    element_Pick_Pose = "Pose_E%i_%i_%i" % (element, userdata.faulty_device, userdata.faulty_device+1)
                else:
                    element_Pick_Pose = "Pose_E%i_%i_%i" % (element, userdata.faulty_device-1, userdata.faulty_device)

                element_Place_Pose = self._searchWarehousePose(userdata, element, forPlacePose=True)

                # Check if the operative robot can reach both poses. If it can, append the poses to the right lists.
                if (self._checkPose(nextRobot, element_Pick_Pose, "Element") and
                    self._checkPose(nextRobot, element_Place_Pose, "Warehouse")):
                    pick_Poses.append(element_Pick_Pose)
                    place_Poses.append(element_Place_Pose)
                else:
                    self._solutionFound = False
                    break

            # Define the pick pose and the place pose for the operative robot for inserting the new and correct elements in the case.
            # Note that the state parameter _new_color_code is needed only to keep track of which element needs to be inserted in this operation
            # of searching pick pose and place pose, but it isn't a real color code of the product.
            # This is the color code remaining after all the elements that need to be removed are removed.
            self._new_color_code = userdata.color_code_detected[: min(elementsToRemove)-1]

            for i in range(userdata.n_element - len(self._new_color_code)):
                element_Pick_Pose = self._searchWarehousePose(userdata, forAfterRemovedElements=True)

                # Note that the helper robot places and rotates the product for easier removal of the element by the operative robot.
                if userdata.faulty_device != userdata.n_line_device: 
                    element_Place_Pose = "Pose_E_Top_%i_%i_Rotated" % (userdata.faulty_device, userdata.faulty_device+1)
                else:
                    element_Place_Pose = "Pose_E_Top_%i_%i_Rotated" % (userdata.faulty_device-1, userdata.faulty_device)

                # If in the warehouse exist an element of the correct color, so exist a pick pose, then
                # the _new_color_code is updated like the element was really inserted.  
                if element_Pick_Pose != None:
                    posColorToAdd = len(self._new_color_code)
                    self._new_color_code.append(userdata.color_code[posColorToAdd])

                # Check if the operative robot can reach both poses. If it can, append the poses to the right lists.
                if (self._checkPose(nextRobot, element_Pick_Pose, "Warehouse") and
                    self._checkPose(nextRobot, element_Place_Pose, "Element")):
                    pick_Poses.append(element_Pick_Pose)
                    place_Poses.append(element_Place_Pose)
                else:
                    self._solutionFound = False
                    break
            
            # Check if a solution has been found. In that case, create the command for the operative robot and publish 
            # the first message for the helper robot.
            if self._solutionFound:
                command = WrongAssemblySolution()
                command.deviceNumber = self._nextRobot
                command.assistantRobot = False
                command.nOperations =  len(pick_Poses)
                command.pickPoses = pick_Poses
                command.placePoses = place_Poses
                self._commands2 = command

                self._pub.publish(self._topicPub, self._commands1[0])
                del self._commands1[0]

        # Remove the last message on the topic if the connection is successful and clearing is enabled.
        if self._connected and self._clear and self._sub.has_msg(self._topicSub):
            self._sub.remove_last_msg(self._topicSub)

    # Method that creates a list of elements of the product that need to be
    # removed to correct the product.
    def _findElementToRemove(self, userdata):
        pos_difference = []
        
        # Find the positions of the elements in the color code detected that don't correspond with
        # the desired color code.
        for i in range(len(userdata.color_code_detected)):
            if userdata.color_code_detected[i] != userdata.color_code[i]:
                pos_difference.append(i+1)
        
        elementsToRemove = []
        # The elements need to be removed from the highest position to the lowest,
        # so the pos_difference needs to be reversed.
        pos_difference.reverse()

        # From the pos_difference, find all the elements that need to be removed even if they are correct.
        # This is needed because if the product has a wrong element at position i, but at position i+1 the element
        # is correct, even the i+1 element needs to be removed to allow the removal of the i element.
        for i in range(len(pos_difference)):
            # If the position difference is the top element, only this is added to the elementsToRemove list;
            # otherwise, all the elements between the top element and the position difference are added to
            # the elementsToRemove list.
            if pos_difference[i] == len(userdata.color_code_detected)-i:
                elementsToRemove.append(pos_difference[i])
            else:
                for k in range(len(userdata.color_code_detected)-i, pos_difference[i]-1, -1):
                    elementsToRemove.append(k)

        return elementsToRemove
    
    # Method for selecting the robots for a dual robot corrective operation.
    # Note that the wrong assembly error can only be resolved by a robot and not other devices.
    # The faulty robot isn't excluded from the solution. If it is the last robot, 
    # it is included in the next robot search; otherwise, it is included in the previous robot search.
    # The robots are searched in a way that, if a solution exists, the product finds itself between the two robots.
    def _selectRobots(self, userdata):
        if userdata.faulty_device == userdata.n_line_device:
            i = 0
            k = 1
        else:
            i = 1
            k = 0

        for n_device in range(userdata.faulty_device+i, userdata.n_line_device+1):
            self._nextRobot = self._isRobot(n_device)
            if self._nextRobot != None:
                break
        
        for n_device in range(userdata.faulty_device-k, 0, -1):
            self._previousRobot = self._isRobot(n_device)
            if self._previousRobot != None:
                break
    
    # Method for selecting the robot for a single robot corrective operation.
    # Note that the wrong assembly error can only be resolved by a robot and not other devices. 
    # The faulty robot is excluded from the solution. If it is the last robot, 
    # the robot for the solution is searched in the previous robot from this one.
    def _selectRobot(self, userdata):
        if userdata.faulty_device != userdata.n_line_device:
            # Iterate from the next device to the end to find a suitable robot.
            for n_device in range(userdata.faulty_device+1, userdata.n_line_device+1):
                self._singleRobot = self._isRobot(n_device)
                if self._singleRobot != None:
                    break
        else:
            # Iterate from the previous device to find a suitable robot.
            for n_device in range(userdata.faulty_device-1, 0, -1):
                self._singleRobot = self._isRobot(n_device)
                if self._singleRobot != None:
                    break
    
    # Method that returns the number of the device in the production line only
    # if it is a robot and not another type of device.
    def _isRobot(self, n_device):
        device_name = "device_%i" % n_device
        device = DevicePoses[device_name].value
        if device["type"] == "robot":
            return n_device

    # Method that verifies if a pose is reachable by the specified device.
    # For a better understanding of the checking process, refer to the structure of the DevicePoses module.
    def _checkPose(self, deviceName, poseToReach, type):
        device = DevicePoses[deviceName].value

        if type == "Case":
            for pose in device["final_poses"]:
                if poseToReach == pose:
                    return True
        elif type == "Warehouse":
            for pose in device["initial_poses"]["element_poses"]:
                if poseToReach == pose:
                    return True
        elif type == "Element":
            for pose in device["near_case_poses"]:
                if poseToReach == pose:
                    return True
        
        return False
    
    # Method to select the warehouse position where the robots can find the required element or place the
    # element in an appropriate position based on the warehouse status.
    # The color of the elements is determined based on the type of operation:
    # - If it is for placing in the warehouse the element removed (forPlacePose=True), the color is obtained from the color code detected.
    # - If it is for picking an element from the warehouse when the color code is not respected (forAfterRemovedElements=True),
    #   the color is obtained from the color code based on the _new_color_code variable.
    # - If it is for picking an element from the warehouse when the color code is respected but one or more required elements are missing,
    #   the color is obtained from the color code based on the color_code_detected variable.
    # Furthermore, the status of the warehouse position depends on the type of operation.
    # If it is a placing operation, the position needs to be free (False state);
    # otherwise, for a picking operation, it needs to be occupied by an element (True state).
    def _searchWarehousePose(self, userdata, nElementsToRemove = 0, forAfterRemovedElements=False, forPlacePose = False):
        if forPlacePose:
            color = userdata.color_code_detected[nElementsToRemove-1]
        elif forAfterRemovedElements:
            color = userdata.color_code[len(self._new_color_code)]
        else:
            color = userdata.color_code[len(userdata.color_code_detected)]
            
        warehousePoses = userdata.warehouses[userdata.name_warehouse]
        warehouseNumber = userdata.name_warehouse[-1]
        subPoseName = "Pose_E_W%s_%s" % (warehouseNumber, color)

        # Search for a pose in the warehouse that meets the requirements.
        for poseName, state in warehousePoses.items():
            if (subPoseName in poseName and state != forPlacePose):
                userdata.warehouses[userdata.name_warehouse][poseName] = forPlacePose
                return poseName
            
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