You can build your arduino code without having to run `catkin_make` by 
following these steps:
* `cd` into your build directory
* issue the command `cmake ..` -- this generates a makefile project
* issue the command `make [science | arm | drive | etc..]` where the optional
arguments are packages within the `Arduino/src` directory. If I wanted to 
compile the science and arm package, I would issue `make science arm`.

