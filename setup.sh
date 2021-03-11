### ALIASES ###
# Workspace

alias source-ros2='source /opt/ros/dashing/setup.bash'
alias source-current-ws='source ./install/local_setup.bash'

alias colcon-clean='rm -r build install log'
alias colcon-build='reset; colcon-clean; colcon build --symlink-install && source-current-ws'

# Sourcing
if ! [ "$ROS_DISTRO" == "dashing" ]; then
  echo 'ROS2 not found, sourcing ROS2 and current workspace'
  source-ros2
fi

# Aliases
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
if [ -f $DIR/aliases.sh ]; then
  source $DIR/aliases.sh
fi
