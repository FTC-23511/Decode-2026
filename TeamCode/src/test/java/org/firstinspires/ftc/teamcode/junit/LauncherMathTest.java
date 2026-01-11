package org.firstinspires.ftc.teamcode.junit;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.globals.MathFunctions;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import static org.firstinspires.ftc.teamcode.globals.Constants.GOAL_POSE;
import static org.firstinspires.ftc.teamcode.globals.Constants.MAX_ANGULAR_VELOCITY;
import static org.firstinspires.ftc.teamcode.globals.MathFunctions.distanceToLauncherValues;
import static org.firstinspires.ftc.teamcode.globals.MathFunctions.getHoodAngleFromVelocity;
import static org.junit.jupiter.api.Assertions.*;

import com.seattlesolvers.solverslib.util.InterpLUT;

import java.util.Arrays;
import java.util.List;

public class LauncherMathTest {
    private InterpLUT launcherLUT;
    private InterpLUT inverseLauncherLUT;

    @BeforeEach
    void setUp() {
        final List<Double> launcherInput  = Arrays.asList(-0.01, 0.0, 4.29,   4.76,  5.22,   5.65,   6.06,   6.48,   10.0); // input: velocity (m/s)
        final List<Double> launcherOutput = Arrays.asList(-0.01, 0.0, 1167.0, 1200.0, 1500.0, 1667.0, 1790.0, 1967.0, 2000.0); // output: ticks/s

        launcherLUT = new InterpLUT(
                launcherInput,
                launcherOutput,
                true
        );

        inverseLauncherLUT = new InterpLUT(
                launcherOutput,
                launcherInput,
                true
        );

        launcherLUT.createLUT();
        inverseLauncherLUT.createLUT();
    }

    @Test
    public void lutTest() {
        double val = launcherLUT.get(-10.0);

        System.out.println("launcherLut: " + val);

        assertEquals(-0.01, val);
    }

    @Test
    public void inverseLutTest() {
        double val = inverseLauncherLUT.get(-10.0);

        System.out.println("launcherLut: " + val);

        assertEquals(-0.01, val);
    }

    @Test
    public void mathTest() {
        double distance = 1.5;
        double vel = 1400;
        double ballVel = inverseLauncherLUT.get(vel);
        double adjustedHoodAngle = getHoodAngleFromVelocity(distance, ballVel);

        double[] distances = MathFunctions.distanceToLauncherValues(distance);

        System.out.println("distance:  " + distance);
        System.out.println("ideal launcher values: " + Arrays.toString(distances));
        System.out.println("currentBallVel: " + ballVel);
        System.out.println("corrected launcher values: " + Arrays.toString(new double[]{ballVel, adjustedHoodAngle}));

        assertTrue(!Double.isNaN(adjustedHoodAngle) && !Double.isNaN(ballVel));
    }
}
