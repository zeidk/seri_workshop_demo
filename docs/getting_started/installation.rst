.. _INSTALLATION:


Installation Instructions
===========================

.. admonition:: Requirements
  :class: attention

    The instructions provided on this page are for Ubuntu 20.04 and ROS 2 Galactic. 

    - See the `Ubuntu 20.04 installation instructions <https://phoenixnap.com/kb/install-ubuntu-20-04>`_ for more information.

    - See the `ROS2 Galactic installation instructions <https://docs.ros.org/en/galactic/Installation.html>`_ for more information.

    At the end of this tutorial, you will have the following directory structure:

    .. code-block:: text
        :class: no-copybutton

        ~
        └── dev
            ├── autoware
            ├── carla_simulator
            ├── autoware_carla_control
            └── autoware_carla_launch
        

        4 directories

Carla Simulator
----------------------------

- Carla 0.9.13 is used in the demo.

    - Get one of the two following pre-built binaries from `here <https://github.com/carla-simulator/carla/releases/tag/0.9.13/>`_.

    - Click on one of the following links to download the archive:
        - `CARLA_0.9.13.tar.gz <https://carla-releases.s3.eu-west-3.amazonaws.com/Linux/CARLA_0.9.13.tar.gz>`_
        - `CARLA_0.9.13_RSS.tar.gz <https://carla-releases.s3.eu-west-3.amazonaws.com/Linux/CARLA_0.9.13_RSS.tar.gz>`_
    - Extract the archive to a directory of your choice. We will use the following directory in this example: :file:`~/dev/carla_simulator/`
    - Check the the simulator can run:

        .. code-block:: bash
            :class: highlight

            mkdir -p ~/dev/carla_simulator/
            # Download carla to ~/dev/carla_simulator/
            # Unzip carla to ~/dev/carla_simulator/
            cd ~/dev/carla_simulator/<carla folder> && ./CarlaUE4.sh -quality-level=Epic -prefernvidia

        **Note:** Options to :file:`CarlaUE4.sh` can be found `here <https://carla.readthedocs.io/en/latest/adv_rendering_options/>`_.



Autoware
--------------------------

Autoware (version Galactic) is used in the demo. The following instructions are taken from the `Autoware Source Installation <https://autowarefoundation.github.io/autoware-documentation/galactic/installation/autoware/source-installation/>`_.

Install Git
^^^^^^^^^^^

    .. code-block:: bash
        :class: highlight

        sudo apt-get -y update
        sudo apt-get -y install git

Install Autoware
^^^^^^^^^^^^^^^^

- Clone autowarefoundation/autoware:

    .. code-block:: bash
        :class: highlight

        cd ~/dev
        git clone https://github.com/autowarefoundation/autoware.git -b galactic
        cd autoware

- Pull the dependencies:

    .. code-block:: bash
        :class: highlight

        cd ~/dev/autoware
        mkdir src
        vcs import src < autoware.repos

- Install dependent ROS packages:

    .. code-block:: bash
        :class: highlight

        source /opt/ros/galactic/setup.bash
        cd ~/dev/autoware
        rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO

- Build the workspace:

    .. code-block:: bash
        :class: highlight

        cd ~/dev/autoware
        colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

    **Note:** ``colcon build`` builds all packages in the workspace. This can take a long time (approximately 20-30 min).

    .. admonition:: Requirements
        :class: attention

        :file:`~/dev/autoware` is a ROS 2 workspace. Each time a modification is made to any file in the :file:`src` folder, the workspace must be rebuilt with ``colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release``.
        



Starting the ARIAC Simulator
----------------------------

There are mainly two ways to start the ARIAC simulator.

Default Configuration
~~~~~~~~~~~~~~~~~~~~~

The following command starts ARIAC with the default configuration:

    .. code-block:: console
        :class: highlight

        ros2 launch ariac_gazebo ariac.launch.py

    - The default trial file is :file:`kitting.yaml`, located in `ariac_gazebo/config/trials/ <https://github.com/usnistgov/ARIAC/tree/ariac2023/ariac_gazebo/config/trials>`_

        - **Note:** All trial files must be placed in this folder.
    - The default sensor configuration is :file:`sensors.yaml`, located in `test_competitor/config/ <https://github.com/usnistgov/ARIAC/tree/ariac2023/test_competitor/config>`_

Custom Configuration
~~~~~~~~~~~~~~~~~~~~

- To start ARIAC with a different trial, use the following command:

    .. code-block:: console
        :class: highlight

        ros2 launch ariac_gazebo ariac.launch.py trial_name:=<trial_file>

    Replace :samp:`{<trial_file>}` with the name of a trial file (without the ``.yaml`` extension). **Reminder:** This trial file **MUST** be placed in :file:`ariac_gazebo/config/trials/`.
    
    **Example:** To start ARIAC with :file:`assembly.yaml` trial file, run the following command:

        .. code-block:: console
            :class: highlight

            ros2 launch ariac_gazebo ariac.launch.py trial_name:=assembly

- Competitors will need to create their own competitor package and use their own sensor configuration file.

        - To create a new competitor package, see :ref:`tutorial 1 <TUTORIAL1>`.
        - To use a custom sensor configuration file, create a directory named :file:`config` in your competitor package and place your sensor configuration file in that directory. 

            - Below is an example of competitor package structure with a custom sensor configuration file named :file:`my_sensors.yaml`.

            .. code-block:: text
                :class: no-copybutton
                
                my_competitor_pkg
                ├── CMakeLists.txt
                ├── package.xml
                └── config
                    └── my_sensors.yaml

        - Make sure to edit :file:`CMakelists.txt` in your competitor package to include the :file:`config` directory.

            .. code-block:: cmake

                install(DIRECTORY config
                    DESTINATION share/${PROJECT_NAME}/
                )

        - Start ARIAC with a custom trial and with a custom sensor configuration file by running the following command:

            .. code-block:: console
                :class: highlight

                ros2 launch ariac_gazebo ariac.launch.py competitor_pkg:=<package> sensor_config:=<sensor_file> trial_name:=<trial_file>

            **Example:** To start ARIAC with :file:`assembly.yaml` using :file:`my_sensors.yaml` sensor configuration file (located in :file:`my_competitor_pkg/config`), run the following command:

                .. code-block:: console
                    :class: highlight

                    ros2 launch ariac_gazebo ariac.launch.py competitor_pkg:=my_competitor_pkg sensor_config:=my_sensors trial_name:=assembly


Moving the Robots
-----------------

To verify that the robots can be controlled properly you will need three terminals:

- *terminal 1*: Start the environment.

    .. code-block:: console
        :class: highlight

        ros2 launch ariac_gazebo ariac.launch.py


- *terminal 2*: Start the moveit node.

    .. code-block:: console
        :class: highlight

        ros2 launch ariac_moveit_config ariac_robots_moveit.launch.py

- *terminal 3*: Start the moveit test node.

    .. code-block:: console
        :class: highlight

        ros2 launch test_competitor moveit_test.launch.py


This should start the competition and move each of the robots to the home position. It will also open an RVIZ window showing the robot's planning scene. 


Running the Test Competitor
---------------------------

A test competitor has been created to demonstrate how to complete some of the basic functions (no challenges) of working with the ARIAC environment.
The test competitor has been tested with ``kitting.yaml``, ``assembly.yaml``, ``combined.yaml``, :class: :file:`kitting_assembly.yaml`, and :file:`kitting_combined.yaml`.
There is no guarantee that the test competitor will work with other trials as the goal of the test competitor is to demonstrate how to interface with the ARIAC environment.


The test competitor is located in the `test_competitor <https://github.com/usnistgov/ARIAC/tree/ariac2023/test_competitor>`_ package. To run the test competitor, use the following commands:

- *terminal 1*: Start the environment.

    .. code-block:: console
        :class: highlight

        ros2 launch ariac_gazebo ariac.launch.py trial_name:=<trial_file>


- *terminal 2*: Start the MoveIt node.

    .. code-block:: console
        :class: highlight

        ros2 launch ariac_moveit_config ariac_robots_moveit.launch.py

- *terminal 3*: Start the competitor node.

    .. code-block:: console
        :class: highlight

        ros2 launch test_competitor competitor.launch.py

The test competitor will start the competition, subscribe to camera and orders topics, and complete orders. 
