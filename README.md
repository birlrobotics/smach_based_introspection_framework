
# Package: smach_based_introspection_framework

# Why This Package Exists

We want a finite-state-machine-based control framework for Rethink Robotics Baxter which satisfies the following needs:

1. During motion, by observing sensory data, the robot should be able to detect and classify anomalies, then execute recovery policy accordingly.
1. If the anomaly detected is never seen before, the robot can ask the operator to label the anomaly and demonstrate recovery policy for it.
1. As the operator gives more help, the robot becomes more autonomous.

This framework attempts to meet these needs. 

# What Can This Package Do

Its core functionalities include:

- Record sensory data on a per trial basis.
- Extract samples of successful motions and samples of anomalies from trial recordings. 
- Train models for anomaly detection and classification.
- Provide a console interface to the operator for labelling anomaly and demonstrating recovery policy.
- Upon known anomalies, execute recovery policies accordingly.

# Demo

A simple demo of picking objects can be found in ``` ./test/demo ```. You can follow the steps below to experience how this framework works.

1. Boot The Robot

    Run the follwoing commands to setup baxter (No need to launch simulation if you're using real robot):
    ```bash
    roslaunch baxter_gazebo baxter_world.launch
    rosrun baxter_interface joint_trajectory_action_server.py
    roslaunch baxter_moveit_config baxter_grippers.launch
    ```

1. Collect Enough Successful Trials

    To collect one successful trial, run the following commands:

    ```bash
    cd ./test/demo
    python pick_n_place_runner.py
    ```

    After each trial, you will notice one folder is created in ``` ./introspection_data_folder/experiment_record_folder ```. If the trial is not successful, delete the latest folder. Make sure you collect enough successful trials (>=5) in the folder.

    Since during this step there is no model for anomaly detection, you will notice that baxter head screen shows yellow lights during the motion of each state, which indicates that no introspection model is found.

1. Train Models For Anomaly Detection

    To train models, we need to first extract samples from trial recordings which are effectively rosbag files.

    To process these rosbag files into samples, run:

    ```bash
    cd ./scripts
    python process_experiment_record_to_dataset.py
    ```

    When it's finished, you will notice ``` ./introspection_data_folder/dataset_folder/latest ``` is created which contains samples that can be used to train models.

    Now, to train models using these samples, run:
    ```bash
    cd ./scripts
    python process_dataset_to_models.py
    ```

    When it's finished, you will notice ``` ./introspection_data_folder/model_folder/latest ``` is created which contains models trained. A training log is generated in ```run.log``` there so you can have a look at the training details.

1. Collect Anomalous Trials

    Since anomaly detection models are in place now, the robot now can detect anomalies and ask help from the operator.

    Run more trials and make them anomalous if possible.

    To run one trial:
    ```bash
    cd ./test/demo
    python pick_n_place_runner.py
    ```

    When an anomaly is detected, look at the terminal and you will find an interface to label the anomaly and teach the recovery policy for it.

1. Train Anomaly Classifier

    Since anomalous trials are collected and labeled, we can train classifiers for them:

    ```bash
    cd ./scripts
    python process_experiment_record_to_dataset.py
    ```
    ```bash
    cd ./scripts
    python process_dataset_to_models.py
    ```

1. Done

    Now the robot is able to detect anomalies, classify them and execute recovery policies accordingly.

# Minimal tutorial 

A minimal tutorial on this framework can be:
1. Define smach state as in ```./test/demo/task_states.py```:
    ```python
    class GoToPickPosition(smach.State):
        def __init__(self):
            smach.State.__init__(self,
                                 outcomes=['Successful'])
            self.state_no = 3 # Skill tag
            self.depend_on_prev_state = True # Set this flag accordingly
    
        def before_motion(self): # Things to do before motion
            limb = 'right'
            traj = BreakOnAnomalyTrajectoryClient(limb)
            traj.gripper_open()
    
        def after_motion(self): # Things to do after motion
            limb = 'right'
            traj = BreakOnAnomalyTrajectoryClient(limb)
            traj.gripper_close()
    
        def get_pose_goal(self): # Goal getter, will be reused in executing recovery policy
            return hardcoded_data.pick_object_pose
    
        def determine_successor(self): # Determine next state
            return 'Successful'
    ```
1. Pass in smach state machine as in ```./test/demo/pick_n_place_runner.py```:
    ```python
    from smach_based_introspection_framework.online_part import (
        smach_runner
    )
    if __name__ == '__main__':
        from task_states import assembly_user_defined_sm
        sm = assembly_user_defined_sm() # Assembly state machine in a user-defined way
        smach_runner.run(sm) # Pass in the state machine and it will be run by our framework
    ```

# Installation
## Dependencies

Since this is a catkin package, you can find out full dependencies by running catkin_make. 
Important dependencies are listed below, they are all catkin packages so you can just git clone them into your catkin workspace:

- [birl_sensory_data_manager](https://github.com/birlrobotics/birl_sensory_data_manager)
- [birl_hmm](https://github.com/birlrobotics/birl_hmm)
- [birl_motion_library](https://github.com/birlrobotics/birl_motion_library)
- [birl_baxter_dmp](https://github.com/birlrobotics/birl_baxter_dmp)

# Utilities

## Automatically record PC screen when an experiment starts

1. Assume the PC whose screen you want to record is named ___RecordPC___. Install ```avconv``` on ___RecordPC___ :
	```
	sudo apt-get install libav-tools
	```

2. Join ___RecordPC___ to the ROS network where you're conducting the experiment, in the case of Baxter, run:
	```
	[On "RecordPC", cd to baxter_ws]
	./baxter.sh
	```
	Then, start screen recording service on ___RecordPC___:
	
	```
	rosrun smach_based_introspection_framework start_cam_recording_service.py 
	```

3. Now, you're all set. Run the experiment and when it's finished, you will have a mp4 file on ___RecordPC___. It's saved at ```introspection_data_folder/experiment_video_folder/*```

## step-by-step
1. run those basic operations refer to the 'birl_kitting_experiment' README.MD file;
2. ...




