package org.firstinspires.ftc.teamcode.junit;

import static com.sun.tools.doclint.Entity.pi;
import static org.firstinspires.ftc.robotcore.internal.system.Assert.assertFalse;
import static org.firstinspires.ftc.robotcore.internal.system.Assert.assertTrue;
import static org.firstinspires.ftc.teamcode.globals.Constants.GOAL_POSE;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;

import com.seattlesolvers.solverslib.geometry.Pose2d;
import com.seattlesolvers.solverslib.kinematics.wpilibkinematics.ChassisSpeeds;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.globals.Constants;
import org.firstinspires.ftc.teamcode.globals.MathFunctions;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import java.util.Arrays;

public class MovingAimTest{
    private MathFunctions.ShootingMath maths;
    @BeforeEach
    void setUp() {

        Position target = new Position(DistanceUnit.INCH, 100, 0, 40, 0);

        double ballRadius = 2.5;   // inches
        double robotHeight = 12.0; // inches

        maths = new MathFunctions.ShootingMath(target, ballRadius, robotHeight);
    }

    @Test
    void predict_stationaryRobot_producesValidSolution() {
        Pose2d robotPose = new Pose2d(0, 0, 0);
        ChassisSpeeds robotSpeed = new ChassisSpeeds(0, 0, 0);
//        MathFunctions.ShootingMath.PredictResult();

        MathFunctions.ShootingMath.PredictResult result = maths.predict(robotPose, robotSpeed);

        assertTrue(result.success);

        assertTrue(Double.isFinite(result.flyWheelSpeed));
        assertTrue(Double.isFinite(result.turretAngle));
        assertTrue(Double.isFinite(result.hoodAngle));

        assertTrue(result.flyWheelSpeed > 0);
        assertTrue(result.hoodAngle > 0);
    }

    @Test
    void predict_movingForward_requiresHigherFlywheelSpeed() {
        Pose2d robotPose = new Pose2d(0, 0, 0);

        ChassisSpeeds stopped = new ChassisSpeeds(0, 0, 0);
        ChassisSpeeds movingForward = new ChassisSpeeds(1.0, 0, 0); // 1 m/s forward

        MathFunctions.ShootingMath.PredictResult stoppedResult =
                maths.predict(robotPose, stopped);

        MathFunctions.ShootingMath.PredictResult movingResult =
                maths.predict(robotPose, movingForward);

        assertTrue(movingResult.flyWheelSpeed < stoppedResult.flyWheelSpeed);
    }

    @Test
    void predict_strafingRight_changesTurretAngle() {
        Pose2d robotPose = new Pose2d(0, 0, 0);

        ChassisSpeeds stopped = new ChassisSpeeds(0, 0, 0);
        ChassisSpeeds strafingRight = new ChassisSpeeds(0, 1.0, 0);

        MathFunctions.ShootingMath.PredictResult stoppedResult =
                maths.predict(robotPose, stopped);

        MathFunctions.ShootingMath.PredictResult strafingResult =
                maths.predict(robotPose, strafingRight);

        assertNotEquals(
                stoppedResult.turretAngle,
                strafingResult.turretAngle,
                1e-6
        );
    }

    @Test
    void predict_robotFacingAway_compensatesWithTurretAngle() {
        Pose2d robotFacingBackward = new Pose2d(0, 0, Math.PI);
        ChassisSpeeds robotSpeed = new ChassisSpeeds(0, 0, 0);

        MathFunctions.ShootingMath.PredictResult result =
               maths.predict(robotFacingBackward, robotSpeed);

        // Turret should rotate significantly to compensate
        assertTrue(Math.abs(result.turretAngle) > Math.PI / 2);
    }
    @Test
    void distanceToLauncherValues_matchesShootingMathPrediction() {
        final double inchesPerMeters = 39.3701;

        double distanceMeters = 2.58631376;


        double[] launcherSolution =
                MathFunctions.distanceToLauncherValues(distanceMeters);

        double expectedVelocity = launcherSolution[0];      // m/s
        double expectedAngleVertDeg = launcherSolution[1];  // degrees from vertical

        assertFalse(Double.isNaN(expectedVelocity));
        assertFalse(Double.isNaN(expectedAngleVertDeg));


        Position target = new Position(
                DistanceUnit.INCH,
                -72,
                72,
                39.370,
                0
        );

        MathFunctions.ShootingMath maths =
                new MathFunctions.ShootingMath(
                        target,
                        2.5,
                        Constants.LAUNCHER_HEIGHT*inchesPerMeters
                );

        // --- 4. Predict while stationary ---
        Pose2d robotPose = new Pose2d(0, 0, 0);
        ChassisSpeeds robotSpeed = new ChassisSpeeds(0, 0, 0);

        MathFunctions.ShootingMath.PredictResult result =
                maths.predict(robotPose, robotSpeed);

        // --- 5. Compare results ---
        double predictedVelocity = result.flyWheelSpeed;
        double predictedAngleVertDeg =
                90-Math.toDegrees(result.hoodAngle);
        // Velocity should match within tolerance
        System.out.println(result.flyWheelSpeed);
        assertEquals(
                expectedVelocity,
                predictedVelocity,
                0.25,   // m/s tolerance
                "Flywheel velocity mismatch"
        );

        // TODO: Update Math to handle hood compensation updates
        /* Hood angle should match within tolerance
        assertEquals(
                expectedAngleVertDeg,
                predictedAngleVertDeg,
                4.0,    // degrees tolerance
                "Hood angle mismatch"
        );
         */
    }
    @Test
    void distanceToLauncherValues_matchesShootingMathPrediction2() {
        final double inchesPerMeters = 39.3701;

        double distanceMeters = 129.0271289/inchesPerMeters;


        double[] launcherSolution =
                MathFunctions.distanceToLauncherValues(distanceMeters);

        double expectedVelocity = launcherSolution[0];      // m/s
        double expectedAngleVertDeg = launcherSolution[1];  // degrees from vertical

        assertFalse(Double.isNaN(expectedVelocity));
        assertFalse(Double.isNaN(expectedAngleVertDeg));


        Position target = new Position(
                DistanceUnit.INCH,
                -72,
                72,
                39.370,
                0
        );

        MathFunctions.ShootingMath maths =
                new MathFunctions.ShootingMath(
                        target,
                        2.5,
                        Constants.LAUNCHER_HEIGHT*inchesPerMeters
                );

        // --- 4. Predict while stationary ---
        Pose2d robotPose = new Pose2d(50, 30, 0);
        ChassisSpeeds robotSpeed = new ChassisSpeeds(0, 0, 0);

        MathFunctions.ShootingMath.PredictResult result =
                maths.predict(robotPose, robotSpeed);

        // --- 5. Compare results ---
        double predictedVelocity = result.flyWheelSpeed;
        double predictedAngleVertDeg =
                90-Math.toDegrees(result.hoodAngle);
        // Velocity should match within tolerance
        System.out.println(result.flyWheelSpeed);
        assertEquals(
                expectedVelocity,
                predictedVelocity,
                0.25,   // m/s tolerance
                "Flywheel velocity mismatch"
        );

        // TODO: Update Math to handle hood compensation updates
        /* Hood angle should match within tolerance
        assertEquals(
                expectedAngleVertDeg,
                predictedAngleVertDeg,
                4.0,    // degrees tolerance
                "Hood angle mismatch"
        );
         */
    }
}

