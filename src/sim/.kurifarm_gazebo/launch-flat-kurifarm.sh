cd $(dirname "$0")
#export GAZEBO_MODEL_PATH=$(cd $(dirname "$0") && pwd)/model:$(cd $(dirname "$0") && pwd)/randomTree/tree_model:${GAZEBO_MODEL_PATH}
export GAZEBO_MODEL_PATH=$(cd $(dirname "$0") && pwd)/model:${GAZEBO_MODEL_PATH}
#ros2 launch ros_gz_sim gz_sim.launch.py gz_args:="random_forest.world"
gazebo --verbose world/flat_kurifarm.world
