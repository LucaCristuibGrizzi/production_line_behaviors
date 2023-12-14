#!/usr/bin/env python
import rospy,rostopic
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxySubscriberCached

class CheckIRState(EventState):
    '''
    Wait for the result of the Infrared Sensor check.

    -- topic	string	The topic on which the result of the Infrared Sensor check should be listened to.
    -- clear 	bool    Drop the last message on this topic upon entering to only 
                        handle messages received since this state is active.

    <= done			    Correct check 
    <= failed           Failed operation (Subscriber connection failure)
    '''

    def __init__(self, topic, clear  = False):
        # Declare outcomes.
        super(CheckIRState, self).__init__(outcomes = ['done', 'failed'])

        # Initialize state parameters for later use in the subscriber.
        self._topic = topic
        self._connected = False
        self._clear = clear

        # Attempt to subscribe to the topic and check if the connection was successful.
        if not self._connect():
            Logger.logwarn('Topic %s for state %s not yet available.\n'
                           'Will try again when entering the state...' % (self._topic, self.name))

    def execute(self, userdata):
        # The state will stay active until the correct message on the topic is received
        # or a connection failure occurs.

        # Check if the subscribe was unsuccessful.
        if not self._connected:
            return 'failed'
        
        # Check if the Infrared Sensor has detected the product.
        # If the value of the message is True, it indicates that the product is detected.
        if self._sub.has_msg(self._topic) and self._sub.get_last_msg(self._topic).data:
            return 'done'
            
    def on_enter(self, userdata):
        # Attempt to subscribe to the topic if the connection was not successful during initialization.
        if not self._connected:
            if self._connect():
                Logger.loginfo('Successfully subscribed to previously unavailable topic %s' % self._topic)
            else:
                Logger.logwarn('Topic %s still not available, giving up.' % self._topic)

        # Remove the last message on the topic if the connection is successful and clearing is enabled.
        if self._connected and self._clear and self._sub.has_msg(self._topic):
            self._sub.remove_last_msg(self._topic)

    # Method that tries to establish a connection to the specified topic.
    # Returns True if the connection is successful, otherwise returns False.
    def _connect(self):
        global_topic = self._topic
        if global_topic[0] != '/':
            global_topic = rospy.get_namespace() + global_topic
        msg_type, msg_topic, _ = rostopic.get_topic_class(global_topic)
        if msg_topic == global_topic:
            self._sub = ProxySubscriberCached({self._topic: msg_type})
            self._connected = True
            return True
        return False