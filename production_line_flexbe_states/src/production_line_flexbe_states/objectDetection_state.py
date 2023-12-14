#!/usr/bin/env python
import rospy,rostopic
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyPublisher, ProxySubscriberCached
from std_msgs.msg import String

class ObjectDetectionState(EventState):
    '''
    Detect an object in the specified image zone.

    -- topicPub             string  The topic on which the command to execute should be published.
    -- topicSub             string  The topic on which should be subscribed to receive the results of the check.
    -- objectDetectionSide  string  The side of the image on which the object should be detected.
                                    Specify "L" (Left) or "R" (Right).
    -- faulty_device        int     The number of the device that could generate the error outcome.
    -- clear 	            bool 	Drop the last message on this topic upon entering to only 
                                    handle messages received since this state is active.

    #>  error_type          string  The error type detected.
    #>  faulty_device       int     The number of the faulty device.
                                                                    
    <= done                 The object detection was successful.
    <= failed               Failed operation (Subscriber connection failure).
    <= error                The object detection was unsuccessful (the product is not in the correct side)
    '''

    def __init__(self, topicPub, topicSub, objectDetectionSide, faulty_device, clear = False):
        # Declare outcomes and output_keys.
        super(ObjectDetectionState, self).__init__(outcomes = ['done', 'failed', 'error'],
                                                   output_keys = ['error_type', 'faulty_device'])

        # Initialize state parameters for the publisher and create the publisher.
        self._objectDetectionSide = objectDetectionSide
        self._topicPub = topicPub
        self._pub = ProxyPublisher({self._topicPub: String})

        # Initialize state parameters for later use in the subscriber.
        self._topicSub = topicSub
        self._connected = False
        self._clear = clear
        
        # Initialize state parameter for faulty_device.
        self._faulty_device = faulty_device

        # Attempt to subscribe to the topic and check if the connection was successful.
        if not self._connect():
            Logger.logwarn('Topic %s for state %s not yet available.\n'
                           'Will try again when entering the state...' % (self._topicSub, self.name))
            
    def execute(self, userdata):
         # Check if the subscribe was unsuccessful.
        if not self._connected:
            return 'failed'
        
        # Check if a message has arrived, and if so, process the response to determine the outcome.
        # True -> Object found in the specified zone.
        # False -> Object not found or found in the wrong zone.
        if self._sub.has_msg(self._topicSub):
            response = self._sub.get_last_msg(self._topicSub).data
            if response == True:
                return 'done'
            elif response == False:
                userdata.error_type = "not_arrived"
                userdata.faulty_device = self._faulty_device
                return 'error'

    def on_enter(self, userdata):
        # When the state become active, the command is created and subsequently 
        # it is published on the topic.
        command = String()
        command.data = self._objectDetectionSide
        self._pub.publish(self._topicPub, command)

        # Attempt to subscribe to the topic if the connection was not successful during initialization.
        if not self._connected:
            if self._connect():
                Logger.loginfo('Successfully subscribed to previously unavailable topic %s' % self._topicSub)
            else:
                Logger.logwarn('Topic %s still not available, giving up.' % self._topicSub)

        # Remove the last message on the topic if the connection is successful and clearing is enabled.
        if self._connected and self._clear and self._sub.has_msg(self._topicSub):
            self._sub.remove_last_msg(self._topicSub)

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