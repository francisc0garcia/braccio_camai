Mechanical assembly
===================

Robot arm: Tinkerkit Braccio Robot 
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

One of the main mechanical component of Braccio CamAI is the robot arm: `Tinkerkit Braccio Robot`_.
It has 6 degrees of freedom (DOF), and uses servo-motors controlled by arduino using PWM (Pulse width modulation).
For simplicity, we have excluded two components (Gripper and last Servo-motor) from the original model. Resulting in a 4-DOF arm.
Notice that the URDF models contains all 6-DOF, in case you want to keep them all and build new applications.

Detailed instructions for building the arm are available: `Tinkerkit Braccio Assembly`_ or `Tinkerkit video tutorial`_.

**Important:** Since the servo-motors do not provide any joint-position feedback, it is important to properly calibrate the arm.
Arduino provides the script `testBraccio90.ino`_ that moves the robot arm to the straight position, please check that the arm is straight after running it.


Camera
^^^^^^
Instead of using the last servo-motor and gripper, we have installed a Logitech C270 camera.
It allows us to capture images directly on the `Robot End-effector`_.

To facilitate the camera installation, a 3D-printed base was designed for this project: `Logitech c270 base model`_. We attached the camera using the 3D-printed model and some additional cable ties.

Remember to pass the USB camera cable inside the inner holes of the robot arm, because the Connector is to big for for passing it through after assembly. 

Ideally, any USB webcam can be used for the project, as long as it has linux-compatible drivers. You can check some of there here: `linux-compatible UVC cameras`_.


(optional) 3D-printed base for Raspberry + Arduino
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

We use a 3D-printed `Arduino Raspberry Pi Mount`_ for attaching all electronic components.


.. _Tinkerkit Braccio Robot: https://store.arduino.cc/tinkerkit-braccio
.. _Tinkerkit Braccio Assembly: https://www.arduino.cc/en/Guide/Braccio
.. _Tinkerkit video tutorial: https://www.youtube.com/watch?v=5VkjJXm6bx8
.. _testBraccio90.ino: https://github.com/arduino-org/arduino-library-braccio/blob/master/examples/testBraccio90/testBraccio90.ino
.. _Robot End-effector: https://en.wikipedia.org/wiki/Robot_end_effector
.. _Logitech c270 base model: https://www.thingiverse.com/thing:3682465
.. _linux-compatible UVC cameras: http://www.ideasonboard.org/uvc/#devices
.. _Arduino Raspberry Pi Mount: https://www.thingiverse.com/thing:1190961

