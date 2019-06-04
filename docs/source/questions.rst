Frequent questions
==================

This section presents some common problems and how to solve them!

Installation and compilation problems
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* If you want to grant root access to GPIO for any executable:

.. code-block:: none

    cd devel/lib/robot_micro    # cd to the directory with your node
    sudo chown root:root my_node     # change ownship to root
    sudo chmod a+rx my_node          # set as executable by all
    sudo chmod u+s my_node           # set the setuid bit

* If you want to remove password with sudo for current user:

edit:
.. code-block:: none

    sudo nano /etc/sudoers

add to end file: *user_name ALL=(ALL) NOPASSWD: ALL*

.. code-block:: none

    ubuntu ALL=(ALL) NOPASSWD: ALL

* If you want to grant root access to cameras and serial ports:

create a udev rule file:

.. code-block:: none

    sudo nano /etc/udev/rules.d/40-permissions.rules

insert content:

.. code-block:: none

    KERNEL=="ttyUSB0", MODE="0666"
    KERNEL=="ttyUSB1", MODE="0666"
    KERNEL=="ttyUSB2", MODE="0666"
    KERNEL=="ttyUSB3", MODE="0666"
    KERNEL=="ttyACM0", MODE="0666"
    KERNEL=="video0", MODE="0666"

For Raspberry pi Cam:

.. code-block:: none

    KERNEL=="vchiq", MODE="0666"



