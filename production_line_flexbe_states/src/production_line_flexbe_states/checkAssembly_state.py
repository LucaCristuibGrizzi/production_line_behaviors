#!/usr/bin/env python
import rospy,rostopic
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyPublisher, ProxySubscriberCached
from std_msgs.msg import String

class CheckAssemblyState(EventState):
    '''
    Check that the assembly of the product was execute correctly.

    -- topicPub     string	The topic on which the command to execute should be published.
    -- topicSub     string  The topic on which should be subscribed to receive the results of the check.
    -- nComponents  int     Number of components to check.
    -- clear 	    bool 	Drop the last message on this topic upon entering to only 
                            handle messages received since this state is active.

    ># color_code   list    The color code that defines how the product needs to be assembled.

    #>  error_type          string  The error type detected.
    #>  color_code_detected list    The color code detected by the camera.

    <= done			The check was successful.
    <= failed       Failed operation (Subscriber connection failure).
    <= error        The check was unsuccessful (the product is not assembled correctly).
    '''

    def __init__(self, topicPub, topicSub, nComponents, clear = False):
        # Declare outcomes, input_keys and output_keys.
        super(CheckAssemblyState, self).__init__(outcomes = ['done', 'failed', 'error'],
                                                 input_keys = ['color_code'],
                                                 output_keys = ['color_code_detected', 'error_type'])

        # Initialize state parameters for the publisher and create the publisher.
        self._topicPub = topicPub
        self._pub = ProxyPublisher({self._topicPub: String})

        # Initialize state parameters for later use in the subscriber.
        self._topicSub = topicSub
        self._connected = False
        self._clear = clear

        # Initialize state parameter for nComponents.
        self._nComponents = nComponents

        # Attempt to subscribe to the topic and check if the connection was successful.
        if not self._connect():
            Logger.logwarn('Topic %s for state %s not yet available.\n'
                           'Will try again when entering the state...' % (self._topicSub, self.name))

    def execute(self, userdata):
        # Check if the subscribe was unsuccessful.
        if not self._connected:
            return 'failed'
        
        # Check if a message has arrived and compute its data to verify the product assembly.
        # Return 'done' if the assembly is correct, otherwise return 'error'.
        if self._sub.has_msg(self._topicSub):
            userdata.color_code_detected = self._sub.get_last_msg(self._topicSub).data.split(";")
            if self._checkColors(userdata):
                return 'done'
            else:
                userdata.error_type = "assembly"
                return 'error'
    
    def on_enter(self, userdata):
        # When the state become active, the command is created and subsequently 
        # it is published on the topic.
        command = String()
        command.data = "CHECK"
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
    
    # Method to check if the components are assembled correctly.
    # If the color code detected corresponds with the expected color code for the specified number of components,
    # the product is assembled correctly (returns True); otherwise, the check is unsuccessful (returns False).
    def _checkColors(self, userdata):
        colorCode = userdata.color_code
        colorCodeDetected = userdata.color_code_detected

        if colorCodeDetected[0] == "No color found":
            return False
        elif colorCodeDetected == colorCode[:self._nComponents]:
            return True
        else:
            return False