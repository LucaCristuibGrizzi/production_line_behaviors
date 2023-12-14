#!/usr/bin/env python
from flexbe_core import EventState

class ErrorRecognitionState(EventState):
    '''
    Choose between differents outcomes based on the type of error
    that occured.
    This state is helpful for activating the right solution error state.

    -- outcomes     string[]    A list of all possible outcomes of this state.
                                The outcomes need to have the same name as the error type
                                they are referring to.

    ># error_type   string      Type of error that occurred
    '''

    def __init__(self, outcomes):
        # Declare outcomes and input_keys.
        super(ErrorRecognitionState, self).__init__(outcomes = outcomes,
                                                    input_keys = ['error_type'])
        # Initialize the outcomes
        self._outcomes = outcomes
        
    def execute(self, userdata):
        # Activate the outcome that has the same name as the error occurred
        # in the production line.
        for err in self._outcomes:
            if userdata.error_type == err:
                return err