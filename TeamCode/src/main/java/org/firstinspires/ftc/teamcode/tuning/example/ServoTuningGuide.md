# Servo Tuning Guide

---

### File Setup

1. In `../TeamCode/tuning/servo/sample`, copy either `singleServoTester` or `doubleServoTester` based off whether your mechanism has either 1 or 2 servos.
   - If you are using `doubleServoTester`, make sure to reverse one of the servos if they are facing each other.
2. Paste the file in `../TeamCode/tuning/servo`, and rename the file, ideally to something that has the mechanism/motor name in it.
3. Rename the class name in the code, fix the package name, and change `ExampleRobot` to `Robot`. Then, rename servos as needed to fit your configuration. Finally, uncomment `@Photon`, `@Config`, and ``@TeleOp`

---

### Tuning Process

1. Connect to robot, deploy, and open FTC Dashboard if you are choosing to tune with it (HIGHLY RECOMMENDED!!!) instead of gamepad. Initialize and run the program.
2. If you are using FTC Dashboard, select `USE_DASHBOARD`.
3. Change `CENTER_SERVO_POS` if you are using 1 servo to adjust its position, or `LEFT_SERVO_POS`/`RIGHT_SERVO_POS` if you are using 2 servos.
4. If you are using 2 servos, and want to move both at the same time, unselect `MOVE_ONE`, and enter your servo values as needed. To move them both, select `MOVE_BOTH`, which will move both servos to their positions. This will also automatically unselect `MOVE_BOTH`
5. Once done, make sure to WRITE THE COEFFICIENTS SOMEWHERE! They will _not_ be saved on FTC Dashboard. I recommend putting them into Android Studio directly, ideally in variables in somewhere like `Globals.java`.