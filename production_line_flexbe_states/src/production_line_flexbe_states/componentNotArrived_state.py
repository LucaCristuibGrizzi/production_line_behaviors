#!/usr/bin/env python
import rospy, rostopic, rospkg
import sys
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyPublisher, ProxySubscriberCached
from std_msgs.msg import String

# Append Custom Path to Execution
package_path = rospkg.RosPack().get_path('production_line_flexbe_states')
sys.path.append(f'{package_path}/src/production_line_flexbe_states/')

# Custom Imports
from Utils.DevicePoses import DevicePoses

class ComponentNotArrivedState(EventState):
    '''
    Manage the error of a component not arrived at a work station in the production line.
    
    -- topicPub         string  The publisher topic for communicating with the devices.
    -- topicSub         string  The subscriber topic for communicating with the devices
    -- clear            bool    Drop the last message on this topic upon entering to only 
                                handle messages received since this state is active.
                                    
    ># faulty_device    int     The number of the faulty device.
    ># n_line_device    int     The number of devices that are present in the production line 
                                (excluding sensors and cameras).

    <= done             The corrective operation has been executed.
    <= failed           Failed operation (Subscriber connection failure).
    <= not_found        There is no solution for the error detected.
    '''
    def __init__(self, topicPub, topicSub, clear = False):
        # Declare outcomes and input_keys.
        super(ComponentNotArrivedState, self).__init__(outcomes = ['done', 'failed', 'not_found'],
                                                       input_keys = ['faulty_device', 'n_line_device'])
        
        # Initialize state parameters for the publisher and create the publisher.
        self._topicPub = topicPub
        self._pub = ProxyPublisher({self._topicPub: String})

        # Initialize state parameters for later use in the subscriber.
        self._topicSub = topicSub
        self._connected = False
        self._clear = clear

        # Create state parameter for later use in the searching and application of the solution.
        self._startPose = None
        self._goalPose = None
        self._device_k = None
        self._device_j = None
        self._solutionFound = False
        self._isDoubleDevice = False
        self._exchangePose = None
        self._isC2Published = False
        self._command1 = String()
        self._command2 = String()

        # Attempt to subscribe to the topic and check if the connection was successful.
        if not self._connect():
            Logger.logwarn('Topic %s for state %s not yet available.\n'
                           'Will try again when entering the state...' % (self._topicSub, self.name))

        
    def execute(self, userdata):
        # Check if the subscribe was unsuccessful.
        if not self._connected:
            return 'failed'
        
        # If a solution for the error detected isn't found then return 'not_found'.
        if not self._solutionFound:
            return 'not_found'

        # If a solution is found, then check if the message of complete operation has arrived.
        if self._sub.has_msg(self._topicSub):
            if self._sub.get_last_msg(self._topicSub).data:
                # If the operation requires two devices to be completed, after receiving the message
                # from the first device, the command for the second device is published.
                if self._isDoubleDevice and not self._isC2Published:
                    self._pub.publish(self._topicPub, self._command2)
                    self._sub.remove_last_msg(self._topicSub)
                    self._isC2Published = True
                else:
                    return 'done'  
    
    def on_enter(self, userdata):
        # The state parameter that indicates a solution found is always reset when this state is activated.
        self._solutionFound = False

        # Choose the device that can resolve the error, define the start and goal pose and search a solution.
        self._selectDevice(userdata)
        self._setPoses(userdata)
        self._searchSolution()

        # If a solution is found, then the command for the first device is published.
        if self._solutionFound:
            self._pub.publish(self._topicPub, self._command1)
        
        # Attempt to subscribe to the topic if the connection was not successful during initialization.
        if not self._connected:
            if self._connect():
                Logger.loginfo('Successfully subscribed to previously unavailable topic %s' % self._topicSub)
            else:
                Logger.logwarn('Topic %s still not available, giving up.' % self._topicSub)

        # Remove the last message on the topic if the connection is successful and clearing is enabled.
        if self._connected and self._clear and self._sub.has_msg(self._topicSub):
            self._sub.remove_last_msg(self._topicSub)
    
    # Method that select the device of the production line that can resolve the error.
    # Generally, these are the one more closer to the faulty device and the subsequent ones,
    # except for the last device of the line and the previous one.
    def _selectDevice(self, userdata):
        if userdata.faulty_device == userdata.n_line_device:
            self._device_k = userdata.faulty_device - 1
            self._device_j = userdata.faulty_device - 2
        elif userdata.faulty_device == userdata.n_line_device - 1:
            self._device_k = userdata.faulty_device + 1
            self._device_j = userdata.faulty_device - 1
        else:
            self._device_k = userdata.faulty_device + 1
            self._device_j = userdata.faulty_device + 2

    # Method that define startPose (where the product is located) and goalPose (where
    # the product should arrive).
    def _setPoses(self, userdata):
        self._startPose = "Pose_C_%i_%i" % (userdata.faulty_device-1, userdata.faulty_device)
        self._goalPose = "Pose_C_%i_%i" % (userdata.faulty_device, userdata.faulty_device+1)

    # Method that search a solution based on the startPose, goalPose and the device selected.
    # The first attempt is to find a solution that use only one device. If it isn't possibile,
    # then it is searched considering even the second device and it searche the existence of 
    # an exchange position between the two devices.
    def _searchSolution(self):
        device_k = "device_%i" % self._device_k
        device_j = "device_%i" % self._device_j

        if self._searchPose(device_k, self._startPose):
            if self._searchPose(device_k, self._goalPose):
                self._solutionFound = True
                self._isDoubleDevice = False
                # The command to publish have this structure because they can be comunicated at all devices.
                self._command1.data = "%i;%s;%s" % (self._device_k, self._startPose, self._goalPose)
            elif self._searchPose(device_j, self._goalPose) and self._searchCommonPose(device_k, device_j):
                self._solutionFound = True
                self._isDoubleDevice = True
                # The command to publish have this structure because they can be comunicated at all devices.
                self._command1.data = "%i;%s;%s" % (self._device_k, self._startPose, self._exchangePose)
                self._command2.data = "%i;%s;%s" % (self._device_j, self._exchangePose, self._goalPose)
            else:
                self._solutionFound = False
        else:
            self._solutionFound = False

    # Method that find if the device indicated can reach the pose specified.
    # For better understaning on how the searching works, look at the structure of the DevicePoses module.
    def _searchPose(self, deviceName, poseToReach):
        device = DevicePoses[deviceName].value

        if device["type"] == "robot":
            for pose in device["initial_poses"]["case_poses"]:
                if poseToReach == pose:
                    return True
        else:
            for pose in device["initial_poses"]:
                if poseToReach == pose:
                    return True
        return False
    
    # Method that search if exist a common pose between the two devices indicated.
    # This pose is needed for exxhange the product between the two devices, so it 
    # can reach the goalPose.
    # For better understaning on how the searching works, look at the structure of the DevicePoses module.
    def _searchCommonPose(self, device1_name, device2_name):
        device1 = DevicePoses[device1_name].value
        device2 = DevicePoses[device2_name].value

        # Note that the type of the device1 hasn't an effect on how the exchange pose is searched.
        if device2["type"] == "robot":
            for pose1 in device1["final_poses"]:
                for pose2 in device2["initial_poses"]["case_poses"]:
                    if pose1 == pose2:
                        self._exchangePose = pose1
                        return True
        else:
            for pose1 in device1["final_poses"]:
                for pose2 in device2["initial_poses"]:
                    if pose1 == pose2:
                        self._exchangePose = pose1
                        return True
        
        return False

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