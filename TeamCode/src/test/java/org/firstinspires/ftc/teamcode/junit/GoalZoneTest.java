package org.firstinspires.ftc.teamcode.junit;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.seattlesolvers.solverslib.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.commandbase.subsystems.Drive;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class GoalZoneTest {
    Pose2d robotPose;

    @BeforeEach
    void setUp() {
    }

    @Test
    public void zoneTest1() {
        robotPose = new Pose2d(0, -17.5 / 2, Math.toRadians(0));

        boolean robotInGoal = Drive.robotInZone(robotPose);

        assertTrue(robotInGoal);
    }

    @Test
    public void zoneTest2() {
        robotPose = new Pose2d(0, -17.6 / 2, Math.toRadians(0));

        boolean robotInGoal = Drive.robotInZone(robotPose);

        assertFalse(robotInGoal);
    }

    @Test
    public void zoneTest3() {
        robotPose = new Pose2d(0, -40, Math.toRadians(90));

        boolean robotInGoal = Drive.robotInZone(robotPose);

        assertFalse(robotInGoal);
    }
}
