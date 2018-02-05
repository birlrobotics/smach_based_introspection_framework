from _core import (
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
import os
import datetime

dir_of_this_script = os.path.dirname(os.path.realpath(__file__))

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
    task_id=None,
    no_state_trainsition_report=False, 
    no_anomaly_detection=False , 
    use_manual_anomaly_signal=False,
):
    import _core
    _core.mode_no_state_trainsition_report = no_state_trainsition_report
    if not no_anomaly_detection:
        listen_HMM_anomaly_signal(use_manual_anomaly_signal)

    from _constant import experiment_record_folder
    if not os.path.isdir(experiment_record_folder):
        os.makedirs(experiment_record_folder)

    from _constant import folder_time_fmt
    experiment_folder = os.path.join(
        experiment_record_folder,
        'experiment_at_%s'%datetime.datetime.now().strftime(folder_time_fmt),
    )
    if not os.path.isdir(experiment_folder):
        os.makedirs(experiment_folder)

    import _experiment_recording_via_rosbag
    o = _experiment_recording_via_rosbag.RosbagRecord(
        os.path.join(experiment_folder, "record.bag"),
        ['/tag_multimodal', '/anomaly_detection_signal']
    )
    o.start()

