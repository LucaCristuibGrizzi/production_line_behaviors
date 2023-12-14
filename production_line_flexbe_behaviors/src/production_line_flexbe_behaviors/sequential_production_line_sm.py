#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from production_line_flexbe_states.checkAssembly_state import CheckAssemblyState as production_line_flexbe_states__CheckAssemblyState
from production_line_flexbe_states.checkIR_state import CheckIRState as production_line_flexbe_states__CheckIRState
from production_line_flexbe_states.colorCode_state import ColorCodeState as production_line_flexbe_states__ColorCodeState
from production_line_flexbe_states.conveyor_state import ConveyorState as production_line_flexbe_states__ConveyorState
from production_line_flexbe_states.main_state import MainState
from production_line_flexbe_states.objectDetection_state import ObjectDetectionState as production_line_flexbe_states__ObjectDetectionState
from production_line_flexbe_states.pickAndPlace_state import PickAndPlaceState as production_line_flexbe_states__PickAndPlaceState
from production_line_flexbe_states.rotatingTable_state import RotatingTableState as production_line_flexbe_states__RotatingTableState
from production_line_flexbe_states.timer_state import TimerState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Mon Oct 23 2023
@author: Luca Crsituib Grizzi
'''
class SequentialProductionLineSM(Behavior):
	'''
	This behavior models the operation of a production line with Niryo robots, working sequentially to assemble a single product. The assembly line is designed without error management.
	'''


	def __init__(self):
		super(SequentialProductionLineSM, self).__init__()
		self.name = 'Sequential Production Line'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:491 y:346, x:737 y:207
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.colors = ["R", "G", "B"]
		_state_machine.userdata.color_code = None
		_state_machine.userdata.robots_operations = None
		_state_machine.userdata.error_type = None
		_state_machine.userdata.speed = None
		_state_machine.userdata.faulty_device = None
		_state_machine.userdata.n_line_device = 5
		_state_machine.userdata.color_code_detected = None
		_state_machine.userdata.n_element = None
		_state_machine.userdata.max_time = 20

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]

		# x:30 y:458, x:130 y:458, x:230 y:458
		_sm_conveyor_belt_run_state_0 = OperatableStateMachine(outcomes=['finished', 'failed', 'error'], output_keys=['error_type', 'speed'])

		with _sm_conveyor_belt_run_state_0:
			# x:137 y:50
			OperatableStateMachine.add('Conveyor Belt Run State',
										production_line_flexbe_states__ConveyorState(topicPub="cmd_run_conveyor", topicSub="conveyor_operation_status", dataPub="RUN", clear=True),
										transitions={'done': 'finished', 'failed': 'Conveyor Belt Stop State', 'error': 'error'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off, 'error': Autonomy.Off},
										remapping={'error_type': 'error_type', 'speed': 'speed'})

			# x:358 y:124
			OperatableStateMachine.add('Conveyor Belt Stop State',
										production_line_flexbe_states__ConveyorState(topicPub="cmd_stop_conveyor", topicSub="conveyor_operation_status", dataPub="ERROR", clear=True),
										transitions={'done': 'failed', 'failed': 'failed', 'error': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off, 'error': Autonomy.Off},
										remapping={'error_type': 'error_type', 'speed': 'speed'})


		# x:648 y:452, x:496 y:454, x:230 y:458, x:330 y:458, x:430 y:458, x:551 y:458
		_sm_check_ir_1 = ConcurrencyContainer(outcomes=['finished', 'failed', 'error'], input_keys=['max_time'], output_keys=['error_type', 'faulty_device'], conditions=[
										('error', [('Max Time', 'done')]),
										('failed', [('Check IR State', 'failed')]),
										('finished', [('Check IR State', 'done')])
										])

		with _sm_check_ir_1:
			# x:422 y:124
			OperatableStateMachine.add('Check IR State',
										production_line_flexbe_states__CheckIRState(topic="/check_IR", clear=True),
										transitions={'done': 'finished', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

			# x:76 y:137
			OperatableStateMachine.add('Max Time',
										TimerState(faulty_device=2),
										transitions={'done': 'error'},
										autonomy={'done': Autonomy.Off},
										remapping={'max_time': 'max_time', 'error_type': 'error_type', 'faulty_device': 'faulty_device'})


		# x:648 y:96, x:130 y:458, x:432 y:467
		_sm_check_ir_2 = OperatableStateMachine(outcomes=['finished', 'failed', 'error'], input_keys=['max_time'], output_keys=['faulty_device', 'error_type', 'speed'])

		with _sm_check_ir_2:
			# x:251 y:71
			OperatableStateMachine.add('Check IR',
										_sm_check_ir_1,
										transitions={'finished': 'finished', 'failed': 'Conveyor Belt Stop State 2', 'error': 'Conveyor Belt Stop State 1'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'error': Autonomy.Inherit},
										remapping={'max_time': 'max_time', 'error_type': 'error_type', 'faulty_device': 'faulty_device'})

			# x:403 y:205
			OperatableStateMachine.add('Conveyor Belt Stop State 1',
										production_line_flexbe_states__ConveyorState(topicPub="/cmd_stop_conveyor", topicSub="/conveyor_operation_status", dataPub="ERROR", clear=True),
										transitions={'done': 'error', 'failed': 'failed', 'error': 'error'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off, 'error': Autonomy.Off},
										remapping={'error_type': 'error_type', 'speed': 'speed'})

			# x:114 y:207
			OperatableStateMachine.add('Conveyor Belt Stop State 2',
										production_line_flexbe_states__ConveyorState(topicPub="/cmd_stop_conveyor", topicSub="/conveyor_operation_status", dataPub="FAILED", clear=True),
										transitions={'done': 'failed', 'failed': 'failed', 'error': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off, 'error': Autonomy.Off},
										remapping={'error_type': 'error_type', 'speed': 'speed'})


		# x:1113 y:761, x:217 y:507, x:378 y:628
		_sm_niryo_production_line_3 = OperatableStateMachine(outcomes=['finished', 'failed', 'error'], input_keys=['robots_operations', 'color_code', 'max_time'], output_keys=['error_type', 'color_code_detected', 'speed', 'faulty_device'])

		with _sm_niryo_production_line_3:
			# x:143 y:37
			OperatableStateMachine.add('R1 PickAndPlace State',
										production_line_flexbe_states__PickAndPlaceState(robot="robot_1", nRobotState=1, topicPub="/cmd_r1", topicSub="/r1_operation_status", clear=True),
										transitions={'done': 'Conveyor Belt Run State', 'failed': 'failed', 'error': 'error'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off, 'error': Autonomy.Off},
										remapping={'robots_operations': 'robots_operations', 'error_type': 'error_type', 'speed': 'speed'})

			# x:1038 y:568
			OperatableStateMachine.add('Check Assembly State 2',
										production_line_flexbe_states__CheckAssemblyState(topicPub="/check_assembly", topicSub="/rs_check_assembly", nComponents=3, clear=True),
										transitions={'done': 'R3 PickAndPlace State 2', 'failed': 'failed', 'error': 'error'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off, 'error': Autonomy.Off},
										remapping={'color_code': 'color_code', 'color_code_detected': 'color_code_detected', 'error_type': 'error_type'})

			# x:635 y:27
			OperatableStateMachine.add('Check IR',
										_sm_check_ir_2,
										transitions={'finished': 'Conveyor Belt Stop State', 'failed': 'failed', 'error': 'error'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'error': Autonomy.Inherit},
										remapping={'max_time': 'max_time', 'faulty_device': 'faulty_device', 'error_type': 'error_type', 'speed': 'speed'})

			# x:365 y:37
			OperatableStateMachine.add('Conveyor Belt Run State',
										_sm_conveyor_belt_run_state_0,
										transitions={'finished': 'Check IR', 'failed': 'failed', 'error': 'error'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'error': Autonomy.Inherit},
										remapping={'error_type': 'error_type', 'speed': 'speed'})

			# x:834 y:54
			OperatableStateMachine.add('Conveyor Belt Stop State',
										production_line_flexbe_states__ConveyorState(topicPub="/cmd_stop_conveyor", topicSub="/conveyor_operation_status", dataPub="STOP", clear=True),
										transitions={'done': 'R2 PickAndPlace State', 'failed': 'failed', 'error': 'error'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off, 'error': Autonomy.Off},
										remapping={'error_type': 'error_type', 'speed': 'speed'})

			# x:1030 y:189
			OperatableStateMachine.add('Object Detection State 1',
										production_line_flexbe_states__ObjectDetectionState(topicPub="/object_detection", topicSub="/rs_object_detection", objectDetectionSide="L", faulty_device=3, clear=False),
										transitions={'done': 'Check Assembly State 1', 'failed': 'failed', 'error': 'error'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off, 'error': Autonomy.Off},
										remapping={'error_type': 'error_type', 'faulty_device': 'faulty_device'})

			# x:1041 y:413
			OperatableStateMachine.add('Object Detection State 2',
										production_line_flexbe_states__ObjectDetectionState(topicPub="/object_detection", topicSub="/rs_object_detection", objectDetectionSide="R", faulty_device=4, clear=True),
										transitions={'done': 'R3 PickAndPlace State 1', 'failed': 'failed', 'error': 'error'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off, 'error': Autonomy.Off},
										remapping={'error_type': 'error_type', 'faulty_device': 'faulty_device'})

			# x:1028 y:111
			OperatableStateMachine.add('R2 PickAndPlace State',
										production_line_flexbe_states__PickAndPlaceState(robot="robot_2", nRobotState=1, topicPub="/cmd_r2", topicSub="/r2_operation_status", clear=True),
										transitions={'done': 'Object Detection State 1', 'failed': 'failed', 'error': 'error'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off, 'error': Autonomy.Off},
										remapping={'robots_operations': 'robots_operations', 'error_type': 'error_type', 'speed': 'speed'})

			# x:1039 y:490
			OperatableStateMachine.add('R3 PickAndPlace State 1',
										production_line_flexbe_states__PickAndPlaceState(robot="robot_3", nRobotState=1, topicPub="/cmd_r3", topicSub="/r3_operation_status", clear=True),
										transitions={'done': 'Check Assembly State 2', 'failed': 'failed', 'error': 'error'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off, 'error': Autonomy.Off},
										remapping={'robots_operations': 'robots_operations', 'error_type': 'error_type', 'speed': 'speed'})

			# x:1041 y:639
			OperatableStateMachine.add('R3 PickAndPlace State 2',
										production_line_flexbe_states__PickAndPlaceState(robot="robot_3", nRobotState=2, topicPub="/cmd_r3", topicSub="r3_operation_status", clear=True),
										transitions={'done': 'finished', 'failed': 'failed', 'error': 'error'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off, 'error': Autonomy.Off},
										remapping={'robots_operations': 'robots_operations', 'error_type': 'error_type', 'speed': 'speed'})

			# x:1037 y:338
			OperatableStateMachine.add('Rotating Table State',
										production_line_flexbe_states__RotatingTableState(topicPub="/cmd_rotating_table", topicSub="/rs_rotating_table", clear=True),
										transitions={'done': 'Object Detection State 2', 'failed': 'failed', 'error': 'error'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off, 'error': Autonomy.Off},
										remapping={'error_type': 'error_type', 'speed': 'speed'})

			# x:1036 y:267
			OperatableStateMachine.add('Check Assembly State 1',
										production_line_flexbe_states__CheckAssemblyState(topicPub="/check_assembly", topicSub="/rs_check_assembly", nComponents=2, clear=True),
										transitions={'done': 'Rotating Table State', 'failed': 'failed', 'error': 'error'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off, 'error': Autonomy.Off},
										remapping={'color_code': 'color_code', 'color_code_detected': 'color_code_detected', 'error_type': 'error_type'})



		with _state_machine:
			# x:54 y:52
			OperatableStateMachine.add('Color Code State',
										production_line_flexbe_states__ColorCodeState(),
										transitions={'done': 'Main State'},
										autonomy={'done': Autonomy.Off},
										remapping={'colors': 'colors', 'color_code': 'color_code'})

			# x:213 y:135
			OperatableStateMachine.add('Main State',
										MainState(),
										transitions={'done': 'Niryo Production Line'},
										autonomy={'done': Autonomy.Off},
										remapping={'robots_operations': 'robots_operations', 'color_code': 'color_code', 'warehouses': 'warehouses'})

			# x:437 y:169
			OperatableStateMachine.add('Niryo Production Line',
										_sm_niryo_production_line_3,
										transitions={'finished': 'finished', 'failed': 'failed', 'error': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'error': Autonomy.Inherit},
										remapping={'robots_operations': 'robots_operations', 'color_code': 'color_code', 'max_time': 'max_time', 'error_type': 'error_type', 'color_code_detected': 'color_code_detected', 'speed': 'speed', 'faulty_device': 'faulty_device'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
