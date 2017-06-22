# Odometry Package for the Mars Rover.
We use `robot_localization` to fuse our sensor and wheel odometry
data into an odometry estimation for the rover.

The base_link of the rover is considered to be the point
between the two mounting points of the middle wheels on the frame
of the rover. This is fixed w.r.t. the chassis of the rover but not
with respect to the wheels of the rover.
