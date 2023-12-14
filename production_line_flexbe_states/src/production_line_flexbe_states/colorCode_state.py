#!/usr/bin/env python
from flexbe_core import EventState, Logger
import random

class ColorCodeState(EventState):
    '''
    Generates a color code from a list of possible colors.
    Note that, in this case, the color code is made-up of three colors.

    ># colors       list    List of possible colors.

    #> color_code   list    Color code of the product to realize.

    <= done			        Color code generated
    '''

    def __init__(self):
        # Declare outcomes, input_keys, and output_keys
        super(ColorCodeState,self).__init__(outcomes=['done'],
                                            input_keys =['colors'],
                                            output_keys = ['color_code'])

    def execute(self, userdata):
        # The color code is already set when the state is active, so
        # it can return the 'done' outcome.
        return 'done'
    
    def on_enter(self, userdata):
        # When the state become active, the color code is set and logged.
        self._setColorCode(userdata)

        Logger.loginfo(userdata.color_code[0])
        Logger.loginfo(userdata.color_code[1])
        Logger.loginfo(userdata.color_code[2])

    # Method that generates the color code from the list of colors.
    # Each color from the list can only appear in the color code once.
    def _setColorCode(self, userdata):
        colorsList = list(userdata.colors)

        firstColor = random.choice(colorsList)
        colorsList.remove(firstColor)

        secondColor = random.choice(colorsList)
        colorsList.remove(secondColor)

        thirdColor = colorsList[0]

        userdata.color_code = [firstColor, secondColor, thirdColor]

