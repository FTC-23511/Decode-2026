package com.seattlesolvers.solverslib.util;

import android.util.Log;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.seattlesolvers.solverslib.geometry.Pose2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.gvf.Path;
import org.firstinspires.ftc.teamcode.gvf.PathData;
import org.firstinspires.ftc.teamcode.gvf.RepulsionPoint;
import org.firstinspires.ftc.teamcode.gvf.Spline;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class TelemetryEx {
    private final Telemetry telemetry;
    private final Canvas canvas;

    private final Map<String, String> dataMap = new HashMap<>();

    public TelemetryEx(Telemetry telemetry) {
        this(telemetry, new Canvas());
    }

    public TelemetryEx(Telemetry telemetry, Canvas canvas) {
        this.telemetry = telemetry;
        this.canvas = canvas;
    }

    public void addData(String caption, Object value) {
        telemetry.addData(caption, value);
        dataMap.put(caption, value.toString());
    }

    public void addLine(String line) {
        telemetry.addLine(line);
        dataMap.put("Line", line);
    }

    public void update() {
        if (!dataMap.isEmpty()) {
            telemetry.update();
            for (Map.Entry<String, String> entry : dataMap.entrySet()) {
                Log.v(entry.getKey(), entry.getValue());
            }

            dataMap.clear();
        }
    }

    /**
     * Draws the current Robot Pose in Green.
     * Draws a circle representing the bot and a line representing the heading.
     *
     * @param robotPose   The current Robot Pose (x, y, heading).
     */
    public void drawRobot(Pose2d robotPose) {
        if (robotPose == null) return;

        canvas.setStroke("green");

        // Draw the Robot Body (Circle)
        canvas.strokeCircle(robotPose.getX(), robotPose.getY(), 9.0);

        // Calculate heading vector to show direction
        double headingX = robotPose.getX() + (9.0 * Math.cos(robotPose.getHeading()));
        double headingY = robotPose.getY() + (9.0 * Math.sin(robotPose.getHeading()));

        // Draw the Heading Line
        canvas.strokeLine(robotPose.getX(), robotPose.getY(), headingX, headingY);
    }

    public void drawPath(Path path, PathData pd) {
        Spline s = path.pathSegments.get(pd.index).spline;

        double n = 100;
        double step = 1/n;
        for (double t = 0; t < 1; t = t + step) {
            canvas.strokeLine(s.getPos(t).getX(), s.getPos(t).getY(), s.getPos(t + step).getX(), s.getPos(t + step).getY());
        }
    }

    public void drawPoints(List<RepulsionPoint> repulsionPoints) {
        for (RepulsionPoint repel : repulsionPoints) {
            canvas.fillCircle(repel.getX(), repel.getY(), 0.5);
        }
    }
}
