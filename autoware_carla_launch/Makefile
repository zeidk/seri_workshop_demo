.PHONY: prepare \
		build \
		clean

build:
	cd external/zenoh-plugin-dds && cargo build --release -p zenoh-bridge-dds
	cd external/zenoh_carla_bridge && cargo build --release
	cd external/zenoh_carla_bridge/carla_agent && poetry install --no-root
	colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

prepare:
	# Install dependencies
	./dependency_install.sh
	# Install necessary ROS package
	vcs import src < autoware_carla.repos
	git submodule update --init --recursive
	./download_map.sh
	rosdep update --rosdistro=${ROS_DISTRO}
	rosdep install -y --from-paths src --ignore-src --rosdistro ${ROS_DISTRO}

clean:
	cd external/zenoh-plugin-dds && cargo clean
	cd external/zenoh_carla_bridge && cargo clean
	rm -rf install log build
	rm -rf external/zenoh_carla_bridge/carla_agent/.venv

docker_clean: clean
	rm -rf rust poetry pyenv

