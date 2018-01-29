from core import (
    get_event_flag,
    set_event_flag,
    AnomalyDiagnosis,
    HumanTeachingRecovery,
    RollBackRecovery,
    write_exec_hist,
    hmm_state_switch_client,
    listen_HMM_anomaly_signal,
)
import copy
from _smach_execute_decorator import smach_execute_decorator
import types
import ipdb

def modify_user_defined_sm(sm):
    import smach
    with sm:
        raw_user_states = copy.deepcopy(sm._states)

        # redirect all NeedRecovery to their respective anomay diagnosis
        for user_state in raw_user_states:
            # Get old definition of exeute, note we only want a
            # unbound pure function, not a bound method.
            original_exec = sm._states[user_state].execute.__func__
            new_exec = smach_execute_decorator(original_exec)
            obj = sm._states[user_state]
            obj.execute = types.MethodType(new_exec, obj)

            state_name = user_state 
            state_transitions = sm._transitions[state_name]
            if "NeedRecovery" in state_transitions:
                state_instance = sm._states[state_name]
                state_no = state_instance.state_no
                ad_state_name = 'AnomalyDiagnosisForSkillNo%s'%state_no
                htr_state_name = "HumanTeachingRecoveryForSkillNo%s"%state_no
                state_transitions["NeedRecovery"] = ad_state_name

                smach.StateMachine.add(
                    ad_state_name,
                    AnomalyDiagnosis(outcomes=["GoToRollBackRecovery", "GoToHumanTeachingRecovery"]),
                    transitions={
                        'GoToRollBackRecovery': 'RollBackRecovery',
                        'GoToHumanTeachingRecovery': htr_state_name,
                    }
                )

                smach.StateMachine.add(
                    htr_state_name,
                    HumanTeachingRecovery(outcomes=["RecoveryDone"]),
                    transitions={
                        'RecoveryDone': state_name,
                    }
                )

        # build Recovery states automatically
        recovery_outcomes = ['RecoveryFailed']
        recovery_state_transitions = {
            'RecoveryFailed':'TaskFailed'
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

def start_instrospection(
    no_state_trainsition_report=False, 
    no_anomaly_detection=False , 
    use_manual_anomaly_signal=False
):
    import core
    core.mode_no_state_trainsition_report = no_state_trainsition_report
    if not no_anomaly_detection:
        listen_HMM_anomaly_signal(use_manual_anomaly_signal)
