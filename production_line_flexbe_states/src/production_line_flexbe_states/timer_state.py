#!/usr/bin/env python
import rospy
from flexbe_core import EventState

class TimerState(EventState):
    '''
    Implements a state that can be used to wait on timed process.

    -- faulty_device    int     The number of the device that could generate the error outcome.

    ># max_time         int     The maximum time after which the 'done' outcome must be activated.

    #>  error_type      string  The error type detected.
    #>  faulty_device   int     The number of the faulty device.

    <= done				Indicates that the wait time has elapsed.
    '''

    def __init__(self, faulty_device):
        # Declare outcomes, input_keys and output_keys.
        super(TimerState, self).__init__(outcomes=['done'],
                                         input_keys = ['max_time'],
                                         output_keys = ['error_type', 'faulty_device'])
        # Initialize state parameter for faulty_device.
        self._faulty_device = faulty_device
        

    def execute(self, userdata):
        # Check if the elapsed time has reached the maximum time.
        # In this case, launch the error 'component not arrived'
        elapsed = rospy.get_rostime() - self._start_time
        if elapsed.to_sec() > float(userdata.max_time):
            userdata.error_type = "not_arrived"
            userdata.faulty_device = self._faulty_device
            return 'done'

    def on_enter(self, userdata):
        # Upon entering the state, save the current time and start waiting.
        self._start_time = rospy.get_rostime()