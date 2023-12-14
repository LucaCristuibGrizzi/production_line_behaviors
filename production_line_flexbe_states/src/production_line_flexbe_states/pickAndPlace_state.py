#!/usr/bin/env python
import rospy,rostopic
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyPublisher, ProxySubscriberCached

# Custom Imports
from production_line_device.msg import PickAndPlace

class PickAndPlaceState(EventState):
    '''
    Control the pick and place operations of the robot.

    -- robot        string	The name of the target pickPose for the robot's operation.
    -- nRobotState  int	    The number indicating the robot's state, to which the state refers,
                            within the behavior (should always be greater than or equal to 1).
    -- topicPub	    string	The publisher topic for communicating with the robot.
    -- topicSub	    string	The subscriber topic for communicating with the robot.
    -- clear 	    bool    Drop the last message on this topic upon entering to only 
                            handle messages received since this state is active.
    
    ># robots_operations    dictonary   Dictionary of robots operations.

    #>  error_type  string  The error type detected.
    #>  speed       int     Percentage of speed communicated by the fault device.

    <= done			Done publishing
    <= failed       Failed operation (Subscriber connection failure).
    <= error        Error in robot operations (in this case, it can only be a slowdown error)
    '''

    def __init__(self, robot, nRobotState, topicPub, topicSub, clear = False):
        # Declare outcomes, input_keys and output_keys.
        super(PickAndPlaceState, self).__init__(outcomes = ['done', 'failed', 'error'],
                                                input_keys = ['robots_operations'],
                                                output_keys = ['error_type', 'speed'])

        # Initialize state parameters for the publisher and create the publisher.
        self._robot = robot
        self._nRobotState = nRobotState
        self._command = PickAndPlace()
        self._topicPub = topicPub
        self._pub = ProxyPublisher({self._topicPub: PickAndPlace})

        # Initialize state parameters for later use in the subscriber.
        self._topicSub = topicSub
        self._connected = False
        self._clear = clear

        # Attempt to subscribe to the topic and check if the connection was successful.
        if not self._connect():
            Logger.logwarn('Topic %s for state %s not yet available.\n'
                           'Will try again when entering the state...' % (self._topicSub, self.name))

    def execute(self, userdata):
        # Check if the subscribe was unsuccessful.
        if not self._connected:
            return 'failed'
        
        # Check if the correct message is received.
        # The robot publishes 'OK' if it has completed the operation; otherwise,
        # it publishes the speed percentage indicating a slowdown.
        if self._sub.has_msg(self._topicSub):
            message = self._sub.get_last_msg(self._topicSub).data
            if message == "OK":
                return 'done'
            else:
                userdata.error_type = "slowdown"
                userdata.speed = int(message)
                return 'error'

    def on_enter(self, userdata):
        # When the state becomes active, the robot command is created using
        # "_setRobotOperations" and subsequently published on the specified topic.
        self._setRobotOperations(userdata)
        self._pub.publish(self._topicPub, self._command)


        # Attempt to subscribe to the topic if the connection was not successful during initialization.
        if not self._connected:
            if self._connect():
                Logger.loginfo('Successfully subscribed to previously unavailable topic %s' % self._topicSub)
            else:
                Logger.logwarn('Topic %s still not available, giving up.' % self._topicSub)

        # Remove the last message on the topic if the connection is successful and clearing is enabled.
        if self._connected and self._clear and self._sub.has_msg(self._topicSub):
            self._sub.remove_last_msg(self._topicSub)

    # Method that creates the command for pick-and-place operations of the robot
    # based on the userdata "robots_operations."
    # To understand the structure of the dictionary "robots_operations," refer to "main_state".
    def _setRobotOperations(self, userdata):
        operations = userdata.robots_operations[self._robot][self._nRobotState-1]
        self._command.nOperations = operations["n_operations"]
        self._command.pickPoses = []
        self._command.placePoses = []

        for i in range(operations["n_operations"]):
            operation_name = "operation_%i" % (i+1)
            self._command.pickPoses.append(operations[operation_name][0])
            self._command.placePoses.append(operations[operation_name][1])

    
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