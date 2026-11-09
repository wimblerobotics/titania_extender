# Development TODO

## Motion control

- [ ] Add motion commands in meters for extension and radians for rotation,
      rather than requiring encoder counts and QPPS.
- [ ] Fix rotation backlash.
- [ ] Revise topic naming.
- [ ] Detect and handle discrepancies between the extension encoder position
      and the ToF sensor measurement.
- [ ] Detect and handle discrepancies between the rotation encoder position
      and the IMU-derived angle.

## Robot integration

- [ ] Create URDF elements for the extender.
- [ ] Port the elevator code from Sigyn, including topic renaming.
- [ ] Enable the top OAK-D camera.
- [ ] Enable the top LIDAR.

## Phase goal: touch a soda can

- [ ] Use the top OAK-D camera to locate a soda can.
- [ ] Verify that the extender can touch the detected can.

Touching the detected soda can is the ultimate goal for this phase of
development.
