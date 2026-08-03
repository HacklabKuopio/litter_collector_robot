all: build

build:
	colcon build --symlink-install

clean:
	-rm -rf build install

# copy production systemd script in place and enable the service
systemd:
	cp systemd/litter_collector_robot.service \
		systemd/lcr-startup-sound.service \
		/lib/systemd/system/
	systemctl enable litter_collector_robot
	systemctl enable lcr-startup-sound

# development environment - start ros nodes in different windows and panes
dev:
	tmuxinator start -p tmuxinator.dev.yml

# production tmux environment - started by systemd/litter_collector_robot.service script
start: prod_start
prod: prod_start
prod_start:
	tmuxinator start -p tmuxinator.prod.yml

stop: prod_stop
prod_stop:
	tmuxinator stop -p tmuxinator.prod.yml

# production will be started as a systemd service, not manually, but this is helpful for debugging
main_launch:
	ros2 launch litter_collector_robot main.launch.xml

.PHONY: build clean systemd dev start prod prod_start stop prod_stop main_launch
