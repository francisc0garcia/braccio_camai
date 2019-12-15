Tutorial: Face detection and tracking
=====================================

Note: This tutorial assumes that the installation was completed successfully.

Components required for face detection and tracking
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Face detection and tracking is composed by a sequence of tasks described as follows:

- Collect and publish images from USB webcam to ROS network.
- Detect faces on images using a face detection module based on Google coral Edge TPU. This module publishes a bounding box for each detected face.
- Track detected faces by generating arm commands, based on detections given as bounding boxes.
- Execute arm commands on Braccio Arm.

These tasks can be executed using the following ROS nodes: 

- **robot_state_publisher**: Publish required transformations (TF) based on URDF model.
- **usb_cam**: Publish camera images to ROS network, using a USB webcam.
- **face_detection**: Subscribe for images and execute the face detection module using Google Coral Edge TPU.
- **DirectJointInterfaceBraccio**: Compute and publish arm commands, based on bounding box messages.
- **rosserial_braccio**: Connects to arduino using rosserial for executing commands on the Braccio arm.

How to execute face detection
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

A launch file containing required nodes is given on: **braccio_camai/launch/face_detection/braccio_driver_face_detection.launch**

It can be executed as follows:

.. code-block:: none

    cd catkin_ws
    source devel/setup.bash
    roscd braccio_camai
    roslaunch  braccio_camai braccio_driver_face_detection.launch


How to test it
^^^^^^^^^^^^^^

Once the launch file is executed, the Braccio arm should move to the initial position. 
If faces can be detected on images, bounding boxes are computed and corresponding arm commands are published.
These commands are executed on the arm using the arduino interface.

A graphical interface based on RVIZ can be use for debugging and testing face detection and tracking.
In a separate terminal, execute the following commands:

.. code-block:: none

    cd catkin_ws
    source devel/setup.bash
    roscd braccio_camai
    roslaunch  braccio_camai braccio_gui_face_detection.launch


