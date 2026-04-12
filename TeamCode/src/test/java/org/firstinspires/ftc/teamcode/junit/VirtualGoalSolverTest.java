package org.firstinspires.ftc.teamcode.junit;

import static org.junit.jupiter.api.Assertions.*;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.BeforeEach;

import org.firstinspires.ftc.teamcode.globals.Constants;
import org.firstinspires.ftc.teamcode.globals.MathFunctions;

import com.seattlesolvers.solverslib.geometry.Pose2d;
import com.seattlesolvers.solverslib.geometry.Rotation2d;
import com.seattlesolvers.solverslib.geometry.Vector2d;
import com.seattlesolvers.solverslib.util.InterpLUT;

import java.util.Arrays;

public class VirtualGoalSolverTest {

    private final Pose2d goalPose = new Pose2d(-72.0, 72.0, new Rotation2d(0));
    private final Pose2d robotPose = new Pose2d(0, 0, new Rotation2d(0));
    private double expectedStationaryAngle;

    private InterpLUT testTimeOfFlightLUT;

    @BeforeEach
    public void setup() {

        testTimeOfFlightLUT = new InterpLUT(
                Arrays.asList(-0.01, 0.0, 5.0, 10.0), // test distances
                Arrays.asList(-0.01, 0.0, 1.5, 3.0),  // test times
                true
        );
        testTimeOfFlightLUT.createLUT();
        // Calculate the TRUE geometric angle from the turret's physical starting
        // position to the goal. This accounts for your offset (1.95977, 0) automatically.
        // Note: rotateBy takes degrees per your previous clarification.
        Vector2d turretPosField = new Vector2d(robotPose.getX(), robotPose.getY())
                .plus(Constants.TURRET_PHYSICAL_OFFSET.rotateBy(robotPose.getRotation().getDegrees()));

        Vector2d toGoal = new Vector2d(goalPose.getX(), goalPose.getY()).minus(turretPosField);

        // This is the baseline angle the turret should have when the robot is not moving.
        expectedStationaryAngle = Math.toDegrees(Math.atan2(toGoal.getY(), toGoal.getX()));
    }

    private void printResults(String testName, MathFunctions.VirtualGoalSolver.ShotSolution sol) {
        System.out.println("--- " + testName + " ---");
        System.out.println("Target Heading (Deg): " + sol.turretGlobalHeading.getDegrees());
        System.out.println("Effective Distance (M): " + sol.effectiveDistance);
        System.out.println("Turret FF (Rad/s): " + sol.turretAngularVelocity);
        System.out.println();
    }

    @Test
    public void testStationary() {
        MathFunctions.VirtualGoalSolver.ShotSolution sol = MathFunctions.VirtualGoalSolver.solve(
                robotPose, new Vector2d(0, 0), 0, goalPose, testTimeOfFlightLUT
        );
        printResults("Stationary", sol);

        // Verification: Static angle should match the geometric calculation
        assertEquals(expectedStationaryAngle, sol.turretGlobalHeading.getDegrees(), 1e-4);
        assertEquals(0.0, sol.turretAngularVelocity, 1e-4);
    }

    @Test
    public void testMovingForward() {
        // Robot moves +X (Forward). Ball inherits forward velocity.
        // Turret must aim more -X (Backward) to compensate.
        // For a goal at (-72, 72), "more backward" results in an angle > 135 deg.
        MathFunctions.VirtualGoalSolver.ShotSolution sol = MathFunctions.VirtualGoalSolver.solve(
                robotPose, new Vector2d(20.0, 0), 0, goalPose, testTimeOfFlightLUT
        );
        printResults("Moving Forward (+X)", sol);

        assertTrue(sol.turretGlobalHeading.getDegrees() > expectedStationaryAngle,
                "Heading should increase to compensate for forward drift");
    }

    @Test
    public void testMovingLeft() {
        // Robot moves +Y (Left). Ball inherits leftward velocity.
        // To compensate, turret must aim Right (-Y relative to goal).
        // In Quadrant 2, aiming "Right" INCREASES the angle toward 180 deg.
        MathFunctions.VirtualGoalSolver.ShotSolution sol = MathFunctions.VirtualGoalSolver.solve(
                robotPose, new Vector2d(0, 20.0), 0, goalPose, testTimeOfFlightLUT
        );
        printResults("Moving Left (+Y)", sol);

        // CORRECTED ASSERTION: Angle should increase from baseline (~135.77)
        assertTrue(sol.turretGlobalHeading.getDegrees() > expectedStationaryAngle,
                String.format("Heading should have INCREASED from %.2f but was %.2f. " +
                                "In Q2, aiming Right increases the degree value.",
                        expectedStationaryAngle, sol.turretGlobalHeading.getDegrees()));
    }

    @Test
    public void testPureSpinningCCW() {
        // Robot center at 0,0. Goal at -72, 72. Spinning CCW (+1 rad/s).
        // Front turret moves Left (+Y). Compensation must be Right (-Y).
        // In Quadrant 2, shifting -Y INCREASES the angle toward 180.
        MathFunctions.VirtualGoalSolver.ShotSolution sol = MathFunctions.VirtualGoalSolver.solve(
                robotPose, new Vector2d(0, 0), 1.0, goalPose, testTimeOfFlightLUT
        );
        printResults("Spinning CCW (+1 rad/s)", sol);

        // CORRECTED ASSERTION: Angle should increase from baseline (~135.77)
        assertTrue(sol.turretGlobalHeading.getDegrees() > expectedStationaryAngle,
                String.format("Heading should have INCREASED from %.2f but was %.2f",
                        expectedStationaryAngle, sol.turretGlobalHeading.getDegrees()));

        // Relative FF should be approx -1.0
        assertEquals(-1.0, sol.turretAngularVelocity, 0.1);
    }

    @Test
    public void testHighSpeedDiagonal() {
        // High speed (30 in/s) in both X and Y directions
        MathFunctions.VirtualGoalSolver.ShotSolution sol = MathFunctions.VirtualGoalSolver.solve(
                robotPose, new Vector2d(30.0, 30.0), 0, goalPose, testTimeOfFlightLUT
        );
        printResults("High Speed Diagonal", sol);

        assertNotNull(sol);
        // Distance should be significantly altered from the stationary ~2.62m
        assertNotEquals(2.6217, sol.effectiveDistance, 0.05);
    }
}