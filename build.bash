colcon clean workspace -y

xacro src/smart_car/urdf/smartcar.urdf.xacro > src/smart_car/urdf/smartcar.urdf
colcon build --packages-select smart_car
. install/setup.sh