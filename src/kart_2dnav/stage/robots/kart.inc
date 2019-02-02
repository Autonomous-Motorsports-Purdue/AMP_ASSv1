# Robot Configuration for Stage Simulation

# The main purpose of this simulation is
# to show the slam mode planning and is not
# the true simulation for the kart

# Laser Ranger
define laser ranger
(
    sensor
    (
        range [0 8]
        fov 180.0
        samples 640
    )
    # generic model properties
    color "black"
    size [0.05 0.05 0.05]
)

define kart position
(
    pose [0.0 0.0 0.0 0.0]

    odom_error [0.03 0.03 999999 999999 999999 0.02]

    size [0.6 0.25 0.40]
    origin [0.2 0.0 0.0 0.0]
    gui_nose 1
    color "red"

    # Kinematics
    drive "car"
    wheelbase 0.4

    # Initialize Sensors
    laser(pose[-0.1 0.0 -0.11 0.0])
)

