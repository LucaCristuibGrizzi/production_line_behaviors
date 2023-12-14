#!/usr/bin/env python
import rospy,rostopic
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyPublisher, ProxySubscriberCached
from std_msgs.msg import String

class ConveyorState(EventState):
    '''
    Control the conveyor belt.

    -- topicPub	    string	The publisher topic for communicating with the conveyor belt.
    -- topicSub	    string	The subscriber topic for communicating with the conveyor belt.
    -- dataPub      string  Data of the message to be published (important for stopping the
                            conveyor with the right procedure in case of an error or using the standard procedure).
    -- clear 	    bool    Drop the last message on this topic upon entering to only 
                            handle messages received since this state is active.
    
    #>  error_type  string  The error type detected.
    #>  speed       int     Percentage of speed communicated by the fault device.                    

    <= done			Done publishing
    <= failed       Failed operation (Subscriber connection failure).
    <= error        Error in conveyor belt operations (in this case, it can only be a slowdown error)
    '''

    def __init__(self, topicPub, topicSub, dataPub, clear = False):
        # Declare outcomes and output_keys.
        super(ConveyorState, self).__init__(outcomes = ['done', 'failed', 'error'],
                                            output_keys = ['error_type', 'speed'])

        # Initialize state parameters for the publisher and create the publisher.
        self._topicPub = topicPub
        self._dataPub = dataPub
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
        # The conveyor belt publishes 'OK' if it has completed the operation;
        # otherwise, it publishes the speed percentage indicating a slowdown.
        if self._sub.has_msg(self._topicSub):
            message = self._sub.get_last_msg(self._topicSub).data
            if message == "OK":
                return 'done'
            else:
                userdata.error_type = "slowdown"
                userdata.speed = int(message)
                return 'error'
            
    def on_enter(self, userdata):
        # When the state becomes active, create and publish the command on the topicPub.
        command = String()
        command.data = self._dataPub
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