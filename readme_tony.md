```

$ tree -I '*.pyc'
.
├── anomaly_classifier
│   ├── Classifier.py # 必看；异常分类
│   ├── __init__.py
│   └── test_data
│       ├── csvs
│       │   ├── No.0 anomaly from experiment_at_2018y05m09d21H36M14S.csv
│       │   ├── No.0 anomaly from experiment_at_2018y05m09d21H41M27S.csv
│       │   ├── No.0 anomaly from experiment_at_2018y05m09d21H54M46S.csv
│       │   ├── No.0 anomaly from experiment_at_2018y05m09d21H57M58S.csv
│       │   ├── No.0 anomaly from experiment_at_2018y05m09d22H00M18S.csv
│       │   ├── No.0 anomaly from experiment_at_2018y05m09d22H03M05S.csv
│       │   ├── No.1 anomaly from experiment_at_2018y05m09d22H00M18S.csv
│       │   └── No.1 anomaly from experiment_at_2018y05m09d22H03M05S.csv
│       └── models
│           ├── classifier_model_NO # 训练好的模型，没有物体
│           └── classifier_model_OS # 训练好的模型，物体滑落
├── anomaly_detector
│   ├── Detectors.py
│   ├── __init__.py
│   ├── log_likelihood_incremental_calculator.py
│   ├── log_likelihood_incremental_calculator.py.orig
│   └── test_data
│       ├── introspection_model
│       └── test_mat.npy
├── data_collection
│   ├── ConvertTagTopicToInterestedVectorProc.py
│   ├── __init__.py
│   ├── plot_offline_signals.py
│   ├── StoreTimeseriesInRedisProc.py
│   ├── StoreVectorToRedisProc.py
│   ├── TagTopicMsgDecorator.py
│   ├── test_data
│   │   ├── experiment_at_2018y04m25d16H36M42S.bag
│   │   └── experiment_at_2018y04m25d16H36M42S.csv
│   └── TimeseriesReceiverProc.py
├── framework_core
│   ├── __init__.py
│   └── states.py
├── __init__.py
├── process_runner 
│   ├── anomaly_classification_process.py # shell运行redis_based_anomaly_classification.py
│   ├── AnomalyDetectionProc.py # shell运行anomaly_detection.py
│   ├── goal_process.py # shell运行goal_topic_and_service.py
│   ├── __init__.py
│   ├── rosbag_process.py #用shell运行rosbag record -O 命令
│   ├── scripts
│   │   ├── anomaly_detection.py # 开启异常检测服务
│   │   ├── anomaly_model_generation.py # 生成异常检测模型
│   │   ├── goal_topic_and_service.py # 得到新的goal ?
│   │   ├── layer_utils.py # ？
│   │   ├── redis_based_anomaly_classification.py # 开启异常分类服务
│   │   ├── tag_multimodal_topic_and_service.py # 发布状态转换
│   │   └── timeseries_publisher.py #将各个sensor的信号整合成vector并用topic发布出去
│   ├── shell_process_runner.py # 将脚步文件用shell运行
│   ├── tag_multimodal_topic_process.py # shell运行tag_multimodal_topic_and_service.py
│   └── timeseries_process.py # shell运行timeseries_publisher.py
├── robot_screen_visualization‘
│   ├── __init__.py
│   └── setter.py # 发送图片给机器人屏幕
├── smach_modifier
│   ├── dmp_execute.py
│   ├── fake_operator.py
│   ├── __init__.py
│   ├── introspection_execute.py
│   ├── modify_user_sm.py
│   ├── quaternion_interpolation.py
│   ├── sos.py
│   ├── test_data
│   │   ├── command_matrix.txt
│   │   └── control_dimensions.pkl
│   └── util.py
└── smach_runner.py


```