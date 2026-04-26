package org.firstinspires.ftc.teamcode.junit;

import static org.firstinspires.ftc.teamcode.globals.Constants.MAX_TURRET_ANGLE;
import static org.firstinspires.ftc.teamcode.globals.Constants.MIN_TURRET_ANGLE;
import static org.firstinspires.ftc.teamcode.globals.Constants.TURRET_SERVO_OFFSET;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.firstinspires.ftc.teamcode.globals.MathFunctions;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class TurretMathTest {
    @BeforeEach
    void setUp() {
    }

    @Test
    public void radToServoMax() {
        double pos = MathFunctions.convertRadianToServoPos(MAX_TURRET_ANGLE) + TURRET_SERVO_OFFSET;

        assertEquals(1.0, pos, 0.01);
    }

    @Test
    public void radToServoMin() {
        double pos = MathFunctions.convertRadianToServoPos(MIN_TURRET_ANGLE) + TURRET_SERVO_OFFSET;

        assertEquals(0.05, pos, 0.01);
    }

    @Test
    public void servoToRadPos() {
        double pos = MathFunctions.convertServoPosToRadian(0.47);

        assertEquals(-0.16755160819145587, pos, 0.01);
    }
}
