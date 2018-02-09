import copy
import types
from smach_based_introspection_framework.online_part.framework_core.states import (
    RollBackRecovery,
)
import introspection_execute

def run(sm):
    import smach
    with sm:
        raw_user_states = copy.deepcopy(sm._states)

        # redirect all NeedRecovery to their respective anomay diagnosis
        for user_state in raw_user_states:
            obj = sm._states[user_state]
            obj.execute = types.MethodType(introspection_execute.execute, obj)

            state_name = user_state 
            state_transitions = sm._transitions[state_name]
            state_transitions["Revert"] = RollBackRecovery.__name__ 

        # build Recovery states automatically
        recovery_outcomes = []
        recovery_state_transitions = {
        }
        for user_state in raw_user_states:
            state_name = user_state 
            recovery_outcomes.append('Reenter_'+state_name)
            recovery_state_transitions['Reenter_'+state_name] = state_name

        smach.StateMachine.add(
			RollBackRecovery.__name__,
			RollBackRecovery(outcomes=recovery_outcomes),
            transitions=recovery_state_transitions
        )

    return sm
