package org.firstinspires.ftc.teamcode.commandbase.subsystems.vision;

import android.graphics.Canvas;
import android.graphics.Color;

import android.graphics.Paint;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Mat;
import org.opencv.core.Rect;

public class RectProcessor implements VisionProcessor {
    private volatile Rect currentRoi = null;
    private final Paint paint;

    public RectProcessor() {
        paint = new Paint();
        paint.setColor(Color.GREEN);
        paint.setStyle(Paint.Style.STROKE);
        paint.setStrokeWidth(5);
    }

    // Call this method from your updateROI() function!
    public void setRoi(Rect roi) {
        this.currentRoi = roi;
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        // No setup needed, but this method MUST exist to satisfy the interface
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        // We aren't doing any detection in this processor, just drawing.
        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        if (currentRoi == null) return;

        // 1. Calculate Scaling
        // The camera is 640x480, but the phone screen might be 1280x720.
        // scaleBmpPxToCanvasPx handles this conversion for us.
        float scale = scaleBmpPxToCanvasPx;

        // 2. Create the Android Rect
        // We strictly use the coordinates passed in, multiplied by the scale.
        android.graphics.RectF drawRect = new android.graphics.RectF(
                currentRoi.x * scale,
                currentRoi.y * scale,
                (currentRoi.x + currentRoi.width) * scale,
                (currentRoi.y + currentRoi.height) * scale
        );

        // 3. Draw
        canvas.drawRect(drawRect, paint);
    }
}