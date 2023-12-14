#!/usr/bin/env python
import rospy,rostopic
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyPublisher, ProxySubscriberCached
from std_msgs.msg import String

class UserSolutionState(EventState):
    '''
    Display a window that prompts the user to confirm whether they have resolved the error.

    -- topicPub	string	The topic on which should be published.
    -- topicSub string  The topic on which should be subscribed.
    -- clear 	bool    Drop the last message on this topic upon entering to only 
                        handle messages received since this state is active.

    ># error_type       string  Type of the error in the production line.
    ># faulty_device    int     Device on the production line where the error occurred.

    <= done             The user has found a solution.
    <= failed           The user hasn't found a solution.
    '''

    def __init__(self, topicPub, topicSub, clear = False):
        # Declare outcomes and input_keys
        super(UserSolutionState, self).__init__(outcomes = ['done', 'failed'],
                                                input_keys = ['error_type', 'faulty_device'])
        
        # Initialize state parameters for the publisher and create the publisher.
        self._topicPub = topicPub
        self._pub = ProxyPublisher({self._topicPub: String})

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
        # "Y" -> Yes, a solution has been found
        # "N" -> No, a solution has not been found
        if self._sub.has_msg(self._topicSub):
            response = self._sub.get_last_msg(self._topicSub).data
            if response == "Y":
                return 'done'
            elif response == "N":
                return 'failed'

    def on_enter(self, userdata):
        # When the state become active, the error string is created and after it
        # is published on the topic.
        error = String()
        error.data = "%s;%s" % (userdata.error_type, userdata.faulty_device)
        self._pub.publish(self._topicPub, error)

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
