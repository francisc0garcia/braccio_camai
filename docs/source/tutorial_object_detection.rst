Tutorial: Object detection and tracking
=======================================

Similar to face detection and tracking, different objects can be deteced and tracked using the Braccion arm.
Supported object classes are taken from **mobilenet_ssd_v2_coco**.

How to execute object detection
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

A launch file containing required nodes is given on: **braccio_camai/launch/face_detection/braccio_driver_object_detection.launch**

It can be executed as follows:

.. code-block:: none

    cd catkin_ws
    source devel/setup.bash
    roscd braccio_camai
    roslaunch  braccio_camai braccio_driver_object_detection.launch

Please notice that, object to be tracked can be parametrized on the ROS node **object_detection**, using the parameter **tracked_object**.
Default object to be tracked is **bottle**.

How to test it
^^^^^^^^^^^^^^

A graphical interface based on RVIZ can be use for debugging and testing object detection and tracking.
In a separate terminal, execute the following commands:

.. code-block:: none

    cd catkin_ws
    source devel/setup.bash
    roscd braccio_camai
    roslaunch  braccio_camai braccio_gui_object_detection.launch

