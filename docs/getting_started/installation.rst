.. _INSTALLATION:


Installation Instructions
===========================

.. admonition:: Requirements
  :class: attention

    - The instructions provided on this page are for Ubuntu 20.04 and ROS 2 Galactic. 

        - See the `Ubuntu 20.04 installation instructions <https://phoenixnap.com/kb/install-ubuntu-20-04>`_ for more information.

        - See the `ROS2 Galactic installation instructions <https://docs.ros.org/en/galactic/Installation.html>`_ for more information.
    - We will use the following directory structure in this tutorial:

        .. code-block:: text
            :class: no-copybutton

            ~
            ├── nist_seri_ws
            |   └── src
            |       └──seri_workshop_demo
            |          └── nist_demo
            ├── autoware_carla_launch
            └── autoware_carla
                ├── autoware
                └── <carla folder>    
    

Nist Demo Package
----------------------------

    - Create the :file:`nist_seri_ws` directory:

        .. code-block:: bash
            :class: no-copybutton

            cd ~
            mkdir -p nist_seri_ws/src

    - Clone the :file:`seri_workshop_demo` package:

        .. code-block:: bash
            :class: no-copybutton

            cd ~/nist_seri_ws/src
            git clone https://github.com/zeidk/seri_workshop_demo.git

    - Build the :file:`seri_workshop_demo` package:

        .. code-block:: bash
            :class: no-copybutton

            cd ~/nist_seri_ws
            colcon build --symlink-install



Carla Simulator
----------------------------

- Carla 0.9.13 is used in the demo.

    - Create the :file:`autoware_carla` directory:

    .. code-block:: bash
        :class: no-copybutton

        cd ~
        mkdir autoware_carla

    - Get one of the two following pre-built binaries from `here <https://github.com/carla-simulator/carla/releases/tag/0.9.13/>`_.

        - Download one of the following pre-built versions in :file:`autoware_carla` :

            - `CARLA_0.9.13.tar.gz <https://carla-releases.s3.eu-west-3.amazonaws.com/Linux/CARLA_0.9.13.tar.gz>`_
            - `CARLA_0.9.13_RSS.tar.gz <https://carla-releases.s3.eu-west-3.amazonaws.com/Linux/CARLA_0.9.13_RSS.tar.gz>`_
    - Extract the archive to a directory of your choice. We will use the following directory in this example: :file:`~/dev/carla_simulator/`
    - Check the the simulator can run:

        .. code-block:: bash
            :class: highlight

            cd ~/autoware_carla/<carla folder> 
            ./CarlaUE4.sh -quality-level=Epic -prefernvidia

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

        cd ~/autoware_carla
        git clone https://github.com/autowarefoundation/autoware.git -b galactic

- Pull the dependencies:

    .. code-block:: bash
        :class: highlight

        cd ~/autoware_carla/autoware
        mkdir src
        vcs import src < autoware.repos

- Install dependent ROS packages:

    .. code-block:: bash
        :class: highlight

        source /opt/ros/galactic/setup.bash
        cd ~/autoware_carla/autoware
        rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO

- Build the workspace:

    .. code-block:: bash
        :class: highlight

        cd ~/autoware_carla/autoware
        colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

    **Note:** ``colcon build`` builds all packages in the workspace. This can take a long time (approximately 20-30 min).

    .. admonition:: Requirements
        :class: attention

        :file:`~/autoware_carla/autoware` is a ROS 2 workspace. Each time a modification is made to any file in the :file:`src` folder, the workspace must be rebuilt with ``colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release``.
        
- Test Autoware:

    - Follow the instructions found `here <https://autowarefoundation.github.io/autoware.auto/AutowareAuto/installation.html#test-your-installation>`_ to test the planning simulation in Autoware.


Carla Autoware Bridge
----------------------------

- Clone the :file:`autoware_carla_launch` package:

        .. code-block:: bash
            :class: no-copybutton

            cd ~
            git clone https://github.com/evshary/autoware_carla_launch.git -b galactic

- Build the :file:`autoware_carla_launch` package:

        .. code-block:: bash
            :class: no-copybutton

            cd ~/autoware_carla_launch
            make clean
            source env.sh
            make prepare
            source ~/autoware_carla/autoware/install/setup.bash
            make build

- Test the bridge:

        .. code-block:: bash
            :class: no-copybutton

            ./CarlaUE4.sh -quality-level=Epic -prefernvidia
            cd ~/autoware_carla_launch
            source env.sh
            ros2 launch autoware_carla_launch autoware_carla.launch.xml


Run Multiple Vehicles
^^^^^^^^^^^^^^^^^^^^^^

- Since running two Autoware will cause port conflict, we need to do some modifications.

    - Modify ``~/autoware_carla/autoware/src/universe/autoware.universe/launch/tier4_planning_launch/launch/scenario_planning/lane_driving/behavior_planning/behavior_planning.launch.py`` (About line 177)

        .. code-block:: python
            :class: highlight

            import random # Add this line
            ...
                {
                    "bt_tree_config_path": [
                        FindPackageShare("behavior_path_planner"),
                        "/config/behavior_path_planner_tree.xml",
                    ],
                    "groot_zmq_publisher_port": random.randint(2000, 4000), # Add this line
                    "groot_zmq_server_port": random.randint(2000, 4000), # Add this line
                    "planning_hz": 10.0,
                },

- Able to spawn a second vehicle into Carla.

    - Modify ``~/autoware_carla_launch/src/autoware_carla_launch/launch/carla_bridge.launch.xml`` (About line 7)

        .. code-block:: xml
            :class: highlight

            -<executable cmd="poetry run python3 main.py --host $(env CARLA_SIMULATOR_IP) --rolename $(env VEHICLE_NAME)" cwd="$(env AUTOWARE_CARLA_ROOT)/external/zenoh_carla_bridge/carla_agent" output="screen" />
            +<executable cmd="poetry run python3 main.py --host $(env CARLA_SIMULATOR_IP) --rolename 'v1' --position 87.687683,145.671295,0.300000,0.000000,90.000053,0.000000" cwd="$(env AUTOWARE_CARLA_ROOT)/external/zenoh_carla_bridge/carla_agent" output="screen" />
            +<executable cmd="poetry run python3 main.py --host $(env CARLA_SIMULATOR_IP) --rolename 'v2' --position 92.109985,227.220001,0.300000,0.000000,-90.000298,0.000000" cwd="$(env AUTOWARE_CARLA_ROOT)/external/zenoh_carla_bridge/carla_agent" output="screen" />

- Spawn two vehicles.

    - Run Carla: 

        .. code-block:: bash
            :class: no-copybutton

            cd ~/autoware_carla/<carla folder>
            ./CarlaUE4.sh -quality-level=Epic -prefernvidia -RenderOffScreen
    
    
    - Run the bridge: 

         .. code-block:: bash
            :class: no-copybutton

            cd ~/autoware_carla_launch
            source env.sh
            ros2 launch autoware_carla_launch carla_bridge.launch.xml
       
    
    - Run Autoware for the first vehicle: 

         .. code-block:: bash
            :class: no-copybutton

            cd ~/autoware_carla_launch
            source env.sh
            ROS_DOMAIN_ID=1 VEHICLE_NAME="v1" ros2 launch autoware_carla_launch autoware_zenoh.launch.xml    
    
    - Run Autoware for the first vehicle: 

         .. code-block:: bash
            :class: no-copybutton

            cd ~/autoware_carla_launch
            source env.sh
            ROS_DOMAIN_ID=2 VEHICLE_NAME="v2" ros2 launch autoware_carla_launch autoware_zenoh.launch.xml    
    