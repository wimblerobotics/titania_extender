# Overall
This is to control the gripper for the robot Titania.

The gripper will consist of:
- An elevator (see https://github.com/wimblerobotics/wr_teensy_boards/blob/main/src/elevator_board.cpp).
This old code controlled both the elevator and the old extender. The new code will control only the elevator and needs to be ported over for Titania, but that port isn't specified here.
- An extender built from https://offsetrobotics.com/product/box-tube-slide-kit-belt-driven-exclusive.
- The slider also includes a side gear mounted to the back of the tubes, driven by another goBilda motor that is able to rotate the slider around the y-axis.
- There will be an actuator attached to the end of the slider. It will probably be a parallel gripper powered by a servo, with additional joints for pitch, yaw and rotate. There may be an additional ability to extend after the pitch joint in order that the whole gripper can reach to the floor.

Control is via topics and actions. In ROS2, the outer namespace is 'gripper'. Within gripper, the next level of name is either 'elevator' or 'slider'. This document only describes /gripper/slider.

Within /gripper/slider the next level of name is 'linear' and 'angular'. These describes two joints. The linear is driven by a goBilda motor https://www.gobilda.com/5204-series-yellow-jacket-planetary-gear-motor-139-1-ratio-80mm-length-8mm-rex-shaft-43-rpm-3-3-5v-encoder/. This drives a pair of goBilda 48 tooth gears https://www.gobilda.com/2302-series-aluminum-mod-0-8-hub-mount-gear-14mm-bore-48-tooth/ which connect to the axel of the slider.

The linear component is driven by a Raspberry Pi 5 which talks to a RoboClaw 2x15A motor controller over USB. The controller's Motor M1 controls the rotation movement and M2 controls the linear movement.

The angular is driven by a goBilda motor https://www.gobilda.com/5203-series-yellow-jacket-planetary-gear-motor-139-1-ratio-24mm-length-8mm-rex-shaft-43-rpm-3-3-5v-encoder/. The motor is attached to the slider via a goBilda u-channel https://www.gobilda.com/1121-series-low-side-u-channel-5-hole-144mm-length/. The plate is attached to the bottom of the slider.. The motor drives a pinion gear https://www.gobilda.com/2303-series-steel-mod-0-8-pinion-gear-8mm-rex-bore-20-tooth/ which, in turn drives a hub mount gear https://www.gobilda.com/2302-series-aluminum-mod-0-8-hub-mount-gear-14mm-bore-100-tooth/. The hub mount gear axis sits directly below the motor axis of the slider and about 70mm below it. The hub mount gear attaches to the elevator and by driving the motor, the whole slider will revolve around the y-axis of the slider at the hub gear axel.

The elevator is https://www.amazon.ca/Stroke-Linear-Motion-Actuator-Position/dp/B07SVP943S?th=1.

The slider has a VL53L4CD ToF sensor that is mounted near the front end of the outer of the nested box tubes and points towards the front. currently, atteched to the forward end of the innermost box tube is a white card. The VL53L4CD measures, then, the extension of the tubes, with some offset. It is intended to enforce the home (fully retracted) and fully extended positions. When linear movement is made, the software must guarantee that no movement happens outside of those two ranges. These ranges are captured in a YAML config file.

Ideally, the linear motion driver will reset the encoder values whenever a home operation is done. The home operation, of course, retracts the gripper until it is within a small tolerance of the home distance as reported by the VL53L4CD. The linear motor has a wheel encoder which is used by the RoboClaw to enforce PID controlled movement. The linear motion driver should stop movement and report an error if the encoder values significantly disagree with the VL53L4CD distance. We need to capture the distance from the pivot point to the home position and then on to the fully extended position so we can correlate that distance with the encoder values.

The angular driver uses a BNO055 IMU to measure the tilt angle of the slider. We have values for the gravity vector when the slider is parallel to the floor. There is a maximum tilt angle, which is when the slider points stright up. The minimum tilt angle has to be computed from the geometry of the robot. As the elevator goes up, the maximum downward tilt also increases.

We need to build a URDF description of the whole gripper assembly so that given the elevator height, slider linear value and slider rotation angle we can compute the 3D position of the center of the innermost tube relative to base_link. Given that Titania's base is a cylinder, this will also allow us to compute the collision linear and angular values that would cause the slider to collide with the top of the robot.

The controls to move the gripper needs to accept meters for the linear value. The fully retracted gripper is 360 mm long but the pivot point along the elevator is 24 mm, so really at full retraction the end of the innermost tube is 336 mm from the center of the vertical component of the elevator. Further, we don't want to fully retract to that position as any error in movement might cause the belt to jam. So reserve a 5 mm distance as the smallest retraction instead of fully retracted. This should be where the 'home' operation moves to and software should enforce that no movement will be at a lesser distance.

The full extension of the slider is 900 mm but we need to, again, reserve a distance at the end so the belt won't jam. The ToF sensor has a bigger error component at that approximately 600mm extension distance, so let's reserve 10mm of the maximum extension. Software should prevent extending beyond that.

There should be a utility program that retracts the linear arm to that 5mm minimum offset, captures the encoder value, extends to the maximum distance minus 10 mm, captures the encoder value, computes the distance per encoder tick, and halts movement with a calibration error if the driver finds that the ToF and encoder values disagree by, say, more than 5%. this probably indicates that the linear movement jammed somewhere. In that case, there needs to be a service 'recalibrate' that when invoked will capture the current goal pose, do a home operation, reset the encoders, and attempt to go back to the goal pose and see if the encoder and ToF agree again. A second failure should halt until some 'reset_fault' service is called to reset that hard error.
