How to install Braccio CamAI package
====================================

The easiest way to use the package is using our RPI3 image (Available soon!).

If you want to compile the package manually, or use a different processing unit (other than RPI), please follow the next steps:

Pre-requirements
^^^^^^^^^^^^^^^^

We use Ubuntu 18.04 running ROS melodic on our Raspberry Pi.
Instructions for installing both are given in:

- Ubuntu 18.04 on RP3: `Ubuntu-18-04-RaspberryPI`_.

- ROS Melodic on RP3: `ROS Melodic Installation`_.

Braccio CamAI package should be compatible with ROS Kinetic, but we haven't tested jet.

Compilation
^^^^^^^^^^^

- Install dependencies:

.. code-block:: none

    sudo apt install ros-melodic-moveit-core ros-melodic-moveit-ros-perception ros-melodic-moveit-ros-planning-interface

(optional) We use `catkin_tools`_. for building the package, you can install it as follows:

.. code-block:: none
    
    pip install --user catkin_tools

- Clone the repository in your ROS workspace:

.. code-block:: none

    cd catkin_ws/src
    git clone https://github.com/francisc0garcia/braccio_camai

- Build the package:

.. code-block:: none

    catkin build --this

If the project build succesfully, we can continue with some tutorials for face and object detection and tracking.

If you have any problem, please check **Frequent questions** section for more information.

.. _Ubuntu-18-04-RaspberryPI: https://wiki.ubuntu.com/ARM/RaspberryPi
.. _ROS Melodic Installation: http://wiki.ros.org/melodic/Installation/Ubuntu
.. _catkin_tools: https://catkin-tools.readthedocs.io/en/latest/installing.html
