colcon clean workspace -y

check_urdf <(xacro src/smartcar_simulation/urdf/smartcar.urdf.xacro)
xacro src/smartcar_simulation/urdf/smartcar.urdf.xacro > src/smartcar_simulation/urdf/smartcar.urdf
colcon build --cmake-args "-DCMAKE_EXPORT_COMPILE_COMMANDS=On"
. install/setup.sh