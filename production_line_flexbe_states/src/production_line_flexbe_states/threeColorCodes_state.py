#!/usr/bin/env python
from flexbe_core import EventState, Logger
import random, itertools

class ThreeColorCodesState(EventState):
    '''
    Generates the color codes for the assembly of three products by the production line.
    For the random generation, keeps into consideration that:
    - Every product must have all three possible colors.
    - In warehouse_1, there is only one element for each color, while in
    warehouse_2, there are two elements for each color.

    ># colors             list  List of possible colors.

    #> first_color_code   list  Color code of the first product to be realized.
    #> second_color_code  list  Color code of the second product to be realized.
    #> third_color_code   list  Color code of the third product to be realized.

    <= done                 Color codes generated.
    '''

    def __init__(self):
        # Declare outcomes, input_keys, and output_keys
        super(ThreeColorCodesState,self).__init__(outcomes=['done'],
                                            input_keys =['colors'],
                                            output_keys = ['first_color_code', 'second_color_code',
                                                           'third_color_code'])

    def execute(self, userdata):
        # The color codes are already set when the state is active, so
        # it can return the 'done' outcome.
        return 'done'
    
    def on_enter(self, userdata):
        #When the state become active, the color codes are set and logged.
        self._setColorCodes(userdata)

        Logger.loginfo("\nFirst Color Code")
        Logger.loginfo(userdata.first_color_code[0])
        Logger.loginfo(userdata.first_color_code[1])
        Logger.loginfo(userdata.first_color_code[2])

        Logger.loginfo("\nSecond Color Code")
        Logger.loginfo(userdata.second_color_code[0])
        Logger.loginfo(userdata.second_color_code[1])
        Logger.loginfo(userdata.second_color_code[2])

        Logger.loginfo("\nThird Color Code")
        Logger.loginfo(userdata.third_color_code[0])
        Logger.loginfo(userdata.third_color_code[1])
        Logger.loginfo(userdata.third_color_code[2])

    # Method that generates the color codes from the list of colors considering
    # the constraint previously described.
    def _setColorCodes(self, userdata):
        colorsList = list(userdata.colors)

        # Create all possible permutations with the three colors in the colors list.
        colorPermutations = list(itertools.permutations(colorsList))

        # Select a random color permutation for the first color code.
        random.shuffle(colorPermutations)
        firstColorCode = list(colorPermutations.pop())

        # Remove color permutations that have at least one color in the
        # same position as the color code of the first product.
        toRemove = []
        for colorCode in colorPermutations:
            for i in range(len(colorCode)):
                if colorCode[i] == firstColorCode[i]:
                    toRemove.append(colorCode)
                    break
        
        for colorCode in toRemove:
            colorPermutations.remove(colorCode)

        # Select a random color permutation from what remains for
        # the second color code.
        random.shuffle(colorPermutations)
        secondColorCode = list(colorPermutations.pop())

        # Remove color permutations that have at least one color in the
        # same position as the color code of the second product.
        toRemove = []
        for colorCode in colorPermutations:
            for i in range(len(colorCode)):
                if colorCode[i] == secondColorCode[i]:
                    toRemove.append(colorCode)
                    break
        
        for colorCode in toRemove:
            colorPermutations.remove(colorCode)

        # Select a random color permutation from what remains for
        # the third color code.
        random.shuffle(colorPermutations)
        thirdColorCode = list(colorPermutations.pop())

        userdata.first_color_code = firstColorCode
        userdata.second_color_code = secondColorCode
        userdata.third_color_code = thirdColorCode