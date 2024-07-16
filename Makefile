all: build

.PHONY: build
build: src
	colcon build
	@echo ''
	@echo 'REMEMBER TO RUN ". install/setup.bash"'

.PHONY: build_debug
build_debug: src
	colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
	@echo ''
	@echo 'REMEMBER TO RUN ". install/setup.bash"'

run: install
	ros2 launch simulation robot.launch.py

debug: install
	ros2 launch simulation robot.launch.py

clean:
	rm -rf build/ install log