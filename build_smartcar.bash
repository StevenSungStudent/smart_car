colcon clean packages --packages-select smartcar_simulation smartcar_msgs -y

check_urdf <(xacro src/smartcar_simulation/urdf/smartcar.urdf.xacro)
xacro src/smartcar_simulation/urdf/smartcar.urdf.xacro > src/smartcar_simulation/urdf/smartcar.urdf
colcon build --packages-select smartcar_simulation smartcar_msgs
. install/setup.sh