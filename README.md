# Smart car
This is the code repository for the smart car project.

## Compilation
The whole project can be built with the following command, given that you are in the project directory.:

```bash
. build_all_smartcar.bash
```
This bash file will clean, build, source and convert the xacro file into a urdf file.

## Running the program
To run the program the following command can be used:

```bash
ros2 launch smartcar_simulation nav2.launch.py
```
