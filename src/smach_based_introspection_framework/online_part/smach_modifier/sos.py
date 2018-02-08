from smach_based_introspection_framework.online_part.framework_core.states import (
    hmm_state_switch_client,
    get_anomaly_t,
)
from smach_based_introspection_framework.online_part.robot_screen_visualization.setter import(
    show_anomaly_detected,
    show_everyhing_is_good,
)
import rospy
from smach_based_introspection_framework.configurables import (
    anomaly_classification_confidence_threshold,
)
from smach_based_introspection_framework._constant import (
    latest_model_folder,
)
from smach_based_introspection_framework.offline_part.process_experiment_record_to_dataset import (
    get_recovery_skill_tag,
)
from sklearn.externals import joblib
import os
import dmp_execute
import glob
import re
from smach_based_introspection_framework._constant import (
    latest_experiment_record_folder,
    anomaly_label_file,
)
def human_teach(state_obj):
    hmm_state_switch_client(-1)
    nominal_tag = state_obj.state_no
    existing_types = []
    prog = re.compile(r'nominal_skill_(\d+)_anomaly_type_(.*)')
    for i in glob.glob(latest_dataset_folder, 'anomaly_data', '*'):
        m = prog.match(os.path.basename(i))
        tag = m.group(1)
        anomaly_type = m.group(2)
        if int(tag) == int(nominal_tag):
            existing_types.append(anomaly_type)
            
    rospy.loginfo("=====================================")
    rospy.loginfo("Help to label this anomaly in skill %s? no/yes (Existing anomaly types: %s)"%(nominal_tag, existing_types))
    while True:
        s = raw_input()
        if s == 'no':
            return 'Unlabeled', False
        elif s == 'yes':
            break
        else:
            rospy.info("input no or yes.")
    rospy.info("type in the label")
    while True:
        raw_label_str = raw_input()
        rospy.info("confirm \"%s\"? yes/no"%raw_label_str)
        s = raw_input()
        if s == 'no':
            rospy.info("try again, type in the label")
            continue 
        elif s == 'yes':
            break
        else:
            rospy.info("input no or yes.")
    
    label = raw_label_str.strip()

    rospy.loginfo("Want to demonstrate how to recover for anomaly type %s in skill %s?"%(label, nominal_tag))
    while True:
        s = raw_input()
        if s == 'no':
            return label, False
        elif s == 'yes':
            break
        else:
            rospy.info("input no or yes.")

    rospy.loginfo("enter \"start\" to begin")
    while True:
        s = raw_input()
        if s == 'start':
            hmm_state_switch_client(-3)
            break
        else:
            rospy.info("input start please.")

    rospy.loginfo("enter \"end\" to end")
    while True:
        s = raw_input()
        if s == 'end':
            hmm_state_switch_client(0)
            break
        else:
            rospy.info("input end please.")
    return label, True

def handle_anomaly(state_obj)
    anomaly_label_file = open(os.path.join(latest_experiment_record_folder, anomaly_label_file) ,'a')
    nominal_tag = state_obj.state_no
    rospy.loginfo("handle_anomaly starts") 

    need_human = False
    while True:
        hmm_state_switch_client(-1)
        anomaly_t = get_anomaly_t()
        show_anomaly_detected()
        rospy.sleep(5)

        sp = rospy.ServiceProxy('AnomalyClassificationService', AnomalyClassificationService)
        try:
            resp = sp(anomaly_t)
        except rospy.ServiceException as exc:
            rospy.logerr("calling AnomalyClassificationService failed")
            raise Exception("calling AnomalyClassificationService failed")

        predicted_label = resp.predicted_label
        predicted_proba = resp.predicted_proba

        rospy.loginfo("anomaly classification resp: %s"%resp) 
        if predicted_proba > anomaly_classification_confidence_threshold  

            dmp_tag = get_recovery_skill_tag(nominal_tag, predicted_label, False)
            rospy.loginfo("dmp_tag : %s"%dmp_tag) 
            if dmp_tag is None:
                rospy.loginfo("no dmp tag found for this anomaly type in this nominal skill") 
                need_human = True
            dmp_model_path = os.path.join(latest_model_folder, 'tag_%s'%dmp_tag, 'dmp_model')
            if not os.path.isfile(dmp_model):
                rospy.loginfo("dmp model not found: %s"%dmp_model_path) 
                need_human = True
            dmp_model = joblib.load(dmp_model_path)
            
            
            anomaly_label_file.write("%s\n"%predicted_label)
            hmm_state_switch_client(dmp_tag)
            if dmp_execute.execute(dmp_model, state_obj.get_pose_goal())                
                break
                hmm_state_switch_client(0)
            else:
                rospy.loginfo("anomaly happened during dmp.") 
        else:
            rospy.loginfo("anomaly classification confidence too low") 
            need_human = True

    if not need_human:
        hmm_state_switch_client(-1)
        label, success = human_teach(state_obj)
        anomaly_label_file.write("%s\n"%label)
        return success

