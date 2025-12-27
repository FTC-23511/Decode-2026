package org.firstinspires.ftc.teamcode.commandbase.commands;


import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS;
import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH;

import com.seattlesolvers.solverslib.geometry.Pose2d;
import com.seattlesolvers.solverslib.kinematics.wpilibkinematics.ChassisSpeeds;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

public class ShootingMath {
    Position target;
    double ballRadius;
    Pose2D robot;
    ChassisSpeeds speed;
    double robotHeight;

    public class PredictResult {
        public boolean success = false;
        public double flyWheelSpeed = 0;
        public double turretAngle = 0;
        public double hoodAngle = 0;
    };

    public ShootingMath(Position target, double ballRadius, double robotHeight) {
        this.target = target;
        this.ballRadius = ballRadius;
        this.robotHeight = robotHeight;
    }


    /**
     * predicts the hood angle, turret angle, and flywheel speed of the robot when aiming while moving
     * uses the robotPose, and robotSpeed.
     */
    public PredictResult predict(Pose2d robotPose, ChassisSpeeds robotSpeed){
        //make a new PredictResult
        PredictResult result = new PredictResult();
        //variables adjusted with robot height and ball radius
        final double targetX = target.x - ballRadius;
        final double targetY = target.y - ballRadius;
        final double targetZ = target.z - robotHeight - ballRadius;
        final double robotX = robotPose.getX();
        final double robotY = robotPose.getY();
        final double inchesPerMeters = 39.3701;

        //mathhhhhh
        //finds the horizontal distance between the robot and the target
        final double distance = Math.sqrt(Math.pow(targetX - robotX,2)+Math.pow(targetY - robotY,2));
        //ball's travel time
        final double time = Math.sqrt(2*((targetZ)+distance)/9.8);
        //horizontal speed of the ball
        final double vh = distance/time;
        //vertical speed of the ball
        final double vz = 9.8*(time) - vh;
        //angle of shooting
        final double alpha = Math.atan((targetY - robotY) / (targetX - robotX));
        //x component of the speed of the ball
        final double vx = Math.sin(alpha)*vh;
        //y component of the speed of the ball
        final double vy = Math.cos(alpha)*vh;
        //x component of the speed of the flywheel
        final double vfx = (vx)-robotSpeed.vxMetersPerSecond*inchesPerMeters;
        //y component of the speed of the flywheel
        final double vfy = (vy)-robotSpeed.vyMetersPerSecond*inchesPerMeters;
        //horizontal component of the speed of the flywheel
        final double vfh = Math.sqrt(Math.pow(vfx,2)+Math.pow(vfy,2));
        //returning turret angle, hood angle, and fly wheel speed
        result.turretAngle = Math.atan(vfy/vfx) - robotPose.getHeading();
        result.hoodAngle = Math.atan(vz/vfh);
        result.flyWheelSpeed = vfh;
        result.success = true;

        return result;
    }
}
