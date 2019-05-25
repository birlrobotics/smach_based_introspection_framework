import copy
import types
from smach_based_introspection_framework.online_part.framework_core.states import (
    RollBackRecovery,
    CheckQTable,
)
import introspection_execute
from smach_based_introspection_framework.online_part.robot_screen_visualization.setter import(
    show_anomaly_detected,
    show_everyhing_is_good,
)
import ipdb

def run(sm):
    import smach

    with sm:
        raw_user_states = copy.deepcopy(sm._states)

        # redirect all NeedRecovery to their respective anomay diagnosis
        for user_state in raw_user_states:
            obj = sm._states[user_state]
            obj.execute = types.MethodType(introspection_execute.execute, obj)
            obj._outcomes.add("Recovery") 

            state_name = user_state 
            state_transitions = sm._transitions[state_name]
            # state_transitions is a dict, add "Revert" transition to it
            state_transitions["Recovery"] = CheckQTable.__name__ 
        # build Recovery states automatically
        recovery_outcomes = ['RecoveryFailed']
        recovery_state_transitions = {
            'RecoveryFailed': 'TaskFailed'
        }
        for user_state in raw_user_states:
            state_name = user_state 
            recovery_outcomes.append('Recovery_with_'+state_name)
            recovery_state_transitions['Recovery_with_'+state_name] = state_name
            
        smach.StateMachine.add(
			CheckQTable.__name__,
			CheckQTable(outcomes=recovery_outcomes),
            transitions=recovery_state_transitions
        )
    return sm
