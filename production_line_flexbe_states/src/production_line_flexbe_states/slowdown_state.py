#!/usr/bin/env python
from flexbe_core import EventState
from flexbe_core.proxy import ProxyPublisher
from std_msgs.msg import Int16
import rospy

class SlowdownState(EventState):
    '''
    Solution of the slowdown error. The new speed is communicated 
    at all the devices of the production line.

    -- topicPub string  The topic on which the speed should be published.

    ># speed    int     Percentage speed to slowdown all the devices of the
                        production line.
                        The value of speed should be included between 1 and 100.

    <= done             Done publishing.
    '''

    def __init__(self, topicPub):
        # Declare outcomes and input_keys.
        super(SlowdownState, self).__init__(outcomes=['done'],
                                            input_keys=['speed'])
        
        # Initialize state parameters for the publisher and create the publisher.
        self._topicPub = topicPub
        self._pub = ProxyPublisher({self._topicPub: Int16})
        
    def execute(self, userdata):
        # Before returning 'done', wait for 200 ms because the robot may have issues 
        # with setting their speed and restarting their operations immediately. 
        rospy.sleep(0.2)
        return 'done'
    
    def on_enter(self, userdata):
        # When the state becomes active, create and publish the command on the topicPub.
        command = Int16()
        command.data = userdata.speed
        self._pub.publish(self._topicPub, command)