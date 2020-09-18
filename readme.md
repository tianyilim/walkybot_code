# walkybot
Programs and drivers for a Hexapod
![Image of robot](https://user-images.githubusercontent.com/50669231/93628698-91796580-fa19-11ea-9494-b16b2b2ba045.jpg)

## Inverse Kinematics
- Leg and Body IK is implemented here based on material from https://oscarliang.com/inverse-kinematics-implementation-hexapod-robots/.
- `hexapod.py` defines classes that describe a hexapod. The `hexapod` class defines motions for a a 6-legged hexapod, which then uses `hex_leg` objects for each individual leg, and subsequently `leg_joint` objects for each individual servo.

Currently, a testbench for RC remote control is implemented with the `RC_test.py` file running on the Rpi, and the `remote_control.py` file running on the host PC (with a gamepad connected).

## Future Extensions
- Negative feedback with accelerometer data
- Dynamic gait selection based on input speed

### GCS to Robot communications
- Create a GUI for robot data:
1. Battery Level, current consumption
2. Controller inputs
3. Accelerometer artificial horizon
4. Selected gait
The input for the direction of the robot will be controlled by a USB Xbox-360 controller, via the Python `inputs` module.