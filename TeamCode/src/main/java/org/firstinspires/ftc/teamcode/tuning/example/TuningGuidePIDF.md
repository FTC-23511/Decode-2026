# PIDF Tuning Guide

---

### File Setup

1. In `../TeamCode/tuning/PIDF/sample`, copy either `singleMotorPIDF` or `doubleMotorPIDF` based off whether your `PIDF` mechanism has either 1 or 2 motors.
    - If you are using `doubleMotorPIDF`, make sure to reverse one of the motors (not encoder) if they are facing each other.
2. Paste the file in `../TeamCode/tuning/PIDF`, and rename the file, ideally to something that has the mechanism/motor name in it.
3. Rename the class name in the code, fix the package name, and change `ExampleRobot` to `Robot`. Then, rename motors and/or encoders as needed to fit your configuration. Finally, uncomment `@Photon`, `@Config`, and ``@TeleOp` 

---

### Tuning Process

1. Make sure `P`, `I`, `D`, & `F` are all `0`!
2. Connect to robot, deploy, and open FTC Dashboard. Initialize and run the program. Select `motorPos` and `setPoint` and graph them.
3. Move mechanism up/down, and make sure encoder increases in positive direction.
    - If it does not, reverse either the motor direction or encoder.
4. Set `setPoint` to a small number, like `200` (depending on max range of mechanism)
5. Set `F` to `0.0001`, and move the mechanism up so that gravity affects it.
6. Increase `F` by increments of `0.0001`, until the mechanism is no longer affected by gravity.
7. Set `P` to `0.001`, and change `setPoint` by increments of roughly `50` to `250` (depending on max range of mechanism).
8. If the mechanism undershoots (most likely scenario), increase `P` by increments of `0.001` until it no longer undershoots. Make sure to change `setPoint` as well after each increase.
9. If the mechanism overshoots (unlikely but still possible), decrease `P` by increments of `0.0001` until it no longer overshoots. Make sure to change `setPoint` as well after each increase.
10. Set `D` to `0.0003`, and change `setPoint` by increments of roughly `50` to `250` (depending on max range of mechanism).
11. Increase `D` by increments of `0.0001` or as needed until the mechanism is smoothly moving. Make sure to change  `setPoint` by increments of roughly `50` to `250` (depending on max range of mechanism) while tuning `D`.
12. Once done, make sure to WRITE THE COEFFICIENTS SOMEWHERE! They will _not_ be saved on FTC Dashboard. I recommend putting them into Android Studio directly, either by changing the `PIDF` parameters from 0 to their tuned values (`I` should still be 0), or by writing them in a comment.