colcon clean workspace -y

check_urdf <(xacro src/smart_car/urdf/smartcar.urdf.xacro)
xacro src/smart_car/urdf/smartcar.urdf.xacro > src/smart_car/urdf/smartcar.urdf
colcon build
. install/setup.sh