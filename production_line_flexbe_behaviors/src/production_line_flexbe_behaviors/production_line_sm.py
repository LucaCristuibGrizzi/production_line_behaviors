#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from production_line_flexbe_states.componentNotArrived_state import ComponentNotArrivedState as production_line_flexbe_states__ComponentNotArrivedState
from production_line_flexbe_states.errorRecognition_state import ErrorRecognitionState as production_line_flexbe_states__ErrorRecognitionState
from production_line_flexbe_states.productionLine_state import ProductionLineState
from production_line_flexbe_states.slowdown_state import SlowdownState as production_line_flexbe_states__SlowdownState
from production_line_flexbe_states.threeColorCodes_state import ThreeColorCodesState
from production_line_flexbe_states.userSolution_state import UserSolutionState as production_line_flexbe_states__UserSolutionState
from production_line_flexbe_states.wrongAssembly_state import WrongAssemblyState as production_line_flexbe_states__WrongAssemblyState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Sat Oct 28 2023
@author: Luca Cristuib Grizzi
'''
class ProductionLineSM(Behavior):
	'''
	This behavior implements error management for a production line with Niryo robots. The production line assembles three products, and the devices operate in parallel.
	'''


	def __init__(self):
		super(ProductionLineSM, self).__init__()
		self.name = 'Production Line'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:359 y:469, x:463 y:484
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.colors = ["R", "G", "B"]
		_state_machine.userdata.first_color_code = None
		_state_machine.userdata.second_color_code = None
		_state_machine.userdata.third_color_code = None
		_state_machine.userdata.faulty_device = 0
		_state_machine.userdata.n_line_device = 5
		_state_machine.userdata.speed = None
		_state_machine.userdata.checkpoint_state = 0
		_state_machine.userdata.color_code = None
		_state_machine.userdata.color_code_detected = None
		_state_machine.userdata.n_element = None
		_state_machine.userdata.error_type = None
		_state_machine.userdata.warehouses = None
		_state_machine.userdata.name_warehouse = None
		_state_machine.userdata.warehouses_init_condition = None

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:97 y:134
			OperatableStateMachine.add('Color Codes State',
										ThreeColorCodesState(),
										transitions={'done': 'Production Line State'},
										autonomy={'done': Autonomy.Off},
										remapping={'colors': 'colors', 'first_color_code': 'first_color_code', 'second_color_code': 'second_color_code', 'third_color_code': 'third_color_code'})

			# x:891 y:293
			OperatableStateMachine.add('Component Not Arrived State',
										production_line_flexbe_states__ComponentNotArrivedState(topicPub="/component_not_arrived", topicSub="/rs_component_not_arrived", clear=True),
										transitions={'done': 'Production Line State', 'failed': 'failed', 'not_found': 'User Solution State'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off, 'not_found': Autonomy.Off},
										remapping={'faulty_device': 'faulty_device', 'n_line_device': 'n_line_device'})

			# x:651 y:134
			OperatableStateMachine.add('Error Recognition State',
										production_line_flexbe_states__ErrorRecognitionState(outcomes=["slowdown", "component_not_arrived", "assembly"]),
										transitions={'slowdown': 'Slowdown State', 'component_not_arrived': 'Component Not Arrived State', 'assembly': 'Wrong Assembly State'},
										autonomy={'slowdown': Autonomy.Off, 'component_not_arrived': Autonomy.Off, 'assembly': Autonomy.Off},
										remapping={'error_type': 'error_type'})

			# x:335 y:136
			OperatableStateMachine.add('Production Line State',
										ProductionLineState(topicPub="/start_production_line", topicSub="/production_line_status", clear=True),
										transitions={'done': 'finished', 'failed': 'failed', 'error': 'Error Recognition State'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off, 'error': Autonomy.Off},
										remapping={'first_color_code': 'first_color_code', 'second_color_code': 'second_color_code', 'third_color_code': 'third_color_code', 'warehouses': 'warehouses', 'error_type': 'error_type', 'speed': 'speed', 'faulty_device': 'faulty_device', 'n_element': 'n_element', 'color_code_detected': 'color_code_detected', 'name_warehouse': 'name_warehouse', 'color_code': 'color_code'})

			# x:874 y:33
			OperatableStateMachine.add('Slowdown State',
										production_line_flexbe_states__SlowdownState(topicPub="/slowdown_error"),
										transitions={'done': 'Production Line State'},
										autonomy={'done': Autonomy.Off},
										remapping={'speed': 'speed'})

			# x:638 y:477
			OperatableStateMachine.add('User Solution State',
										production_line_flexbe_states__UserSolutionState(topicPub="/ask_question", topicSub="/user_response", clear=True),
										transitions={'done': 'Production Line State', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'error_type': 'error_type', 'faulty_device': 'faulty_device'})

			# x:897 y:404
			OperatableStateMachine.add('Wrong Assembly State',
										production_line_flexbe_states__WrongAssemblyState(topicPub="/wrong_assembly", topicSub="/rs_wrong_assembly", clear=True),
										transitions={'done': 'Production Line State', 'failed': 'User Solution State', 'not_found': 'User Solution State'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off, 'not_found': Autonomy.Off},
										remapping={'faulty_device': 'faulty_device', 'n_line_device': 'n_line_device', 'color_code': 'color_code', 'color_code_detected': 'color_code_detected', 'n_element': 'n_element', 'warehouses': 'warehouses', 'name_warehouse': 'name_warehouse'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
