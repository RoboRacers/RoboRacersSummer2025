package org.firstinspires.ftc.teamcode.teleop.LobsterCupTeleop.SystemTest;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class OptimizedPipeline extends OpenCvPipeline {

    public enum TargetColor { RED, BLUE, YELLOW }
    private TargetColor targetColor = TargetColor.BLUE;

    private boolean shouldProcess = false;
    private boolean hasProcessed = false;
    private boolean drawOverlay = true;

    private Point center;
    private double[] hsvPixel;

    private double angle = 0;
    private double distance = 0;
    private double width = 0;
    private double height = 0;
    private double lastArea = 0;
    private double targetAngle = 0;
    private int detectedObjectsCount = 0;

    private static final double MIN_CONTOUR_AREA = 500.0;

    private final Scalar lowerYellow = new Scalar(20, 100, 100);
    private final Scalar upperYellow = new Scalar(30, 255, 255);

    private final Scalar lowerBlue = new Scalar(100, 150, 50);
    private final Scalar upperBlue = new Scalar(140, 255, 255);

    private final Scalar lowerRed1 = new Scalar(0, 120, 70);
    private final Scalar upperRed1 = new Scalar(10, 255, 255);

    private final Scalar lowerRed2 = new Scalar(170, 120, 70);
    private final Scalar upperRed2 = new Scalar(180, 255, 255);

    public void setTargetColor(TargetColor color) {
        this.targetColor = color;
    }

    public void enableOverlay(boolean enabled) {
        this.drawOverlay = enabled;
    }

    public void triggerSnapshot() {
        shouldProcess = true;
        hasProcessed = false;
    }

    public void clearSnapshot() {
        center = null;
        hsvPixel = null;
        angle = 0;
        distance = 0;
        width = 0;
        height = 0;
        lastArea = 0;
        targetAngle = 0;
        detectedObjectsCount = 0;
        hasProcessed = false;
        shouldProcess = false;
    }

    public boolean hasProcessedSnapshot() {
        return hasProcessed;
    }

    @Override
    public Mat processFrame(Mat input) {
        if (!shouldProcess) return input;

        // Local Mat variables to avoid memory leaks
        Mat hsvFrame = new Mat();
        Mat mask = new Mat();
        Mat mask1 = new Mat();
        Mat mask2 = new Mat();
        Mat processedMask = new Mat();
        Mat hierarchy = new Mat();
        List<MatOfPoint> contours = new ArrayList<>();

        try {
            Imgproc.cvtColor(input, hsvFrame, Imgproc.COLOR_RGB2HSV);

            // Generate color mask
            switch (targetColor) {
                case YELLOW:
                    Core.inRange(hsvFrame, lowerYellow, upperYellow, mask);
                    break;
                case BLUE:
                    Core.inRange(hsvFrame, lowerBlue, upperBlue, mask);
                    break;
                case RED:
                    Core.inRange(hsvFrame, lowerRed1, upperRed1, mask1);
                    Core.inRange(hsvFrame, lowerRed2, upperRed2, mask2);
                    Core.bitwise_or(mask1, mask2, mask);
                    break;
            }

            // Morphological cleanup
            Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
            Imgproc.morphologyEx(mask, processedMask, Imgproc.MORPH_OPEN, kernel);
            kernel.release(); // release temp Mat

            // Contour detection
            Imgproc.findContours(processedMask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
            detectedObjectsCount = 0;
            MatOfPoint largestContour = null;
            double largestArea = 0;

            for (MatOfPoint contour : contours) {
                double area = Imgproc.contourArea(contour);
                if (area > MIN_CONTOUR_AREA) {
                    MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());
                    MatOfPoint2f approxCurve = new MatOfPoint2f();
                    double epsilon = 0.02 * Imgproc.arcLength(contour2f, true);
                    Imgproc.approxPolyDP(contour2f, approxCurve, epsilon, true);

                    if (approxCurve.toArray().length <= 6) {
                        detectedObjectsCount++;
                        if (area > largestArea) {
                            largestArea = area;
                            largestContour = contour;
                            width = contour.width();
                            height = contour.height();
                        }
                    }

                    contour2f.release();
                    approxCurve.release();
                }
            }

            lastArea = largestArea;

            if (largestContour != null) {
                RotatedRect rect = Imgproc.minAreaRect(new MatOfPoint2f(largestContour.toArray()));
                center = rect.center;

                angle = rect.angle;
                if (rect.size.width < rect.size.height) {
                    angle += 90;
                }

                if (drawOverlay) {
                    Point[] points = new Point[4];
                    rect.points(points);
                    for (int i = 0; i < 4; i++) {
                        Imgproc.line(input, points[i], points[(i + 1) % 4], new Scalar(70, 50, 50), 2);
                    }
                    Imgproc.circle(input, center, 5, new Scalar(255, 0, 0), -1);
                    Imgproc.putText(input,
                            String.format("Angle: %.2f", angle),
                            new Point(center.x - 50, center.y - 20),
                            Imgproc.FONT_HERSHEY_SIMPLEX,
                            0.8,
                            new Scalar(0, 255, 0),
                            2
                    );
                }

                distance = Math.max(width, height);
                double frameCenterX = input.width() / 2.0;
                targetAngle = Math.atan2(center.x - frameCenterX, input.height());
            }

            // HSV pixel debug
            int cx = input.width() / 2;
            int cy = input.height() / 2;
            hsvPixel = hsvFrame.get(cy, cx);
            if (drawOverlay) {
                Imgproc.circle(input, new Point(cx, cy), 5, new Scalar(0, 255, 0), 5);
            }

        } finally {
            // Release all Mats to avoid memory leaks
            hsvFrame.release();
            mask.release();
            mask1.release();
            mask2.release();
            processedMask.release();
            hierarchy.release();
            for (MatOfPoint c : contours) {
                c.release();
            }
            contours.clear();
        }

        hasProcessed = true;
        shouldProcess = false;
        return input;
    }

    // Accessors
    public double getTargetAngle() { return angle; }

    public double getDistance() { return distance; }

    public double[] getHSVCenter() { return hsvPixel; }

    public double getArea() { return lastArea; }

    public int getDetectedObjectsCount() { return detectedObjectsCount; }

    public Point getCenter() { return center; }
}
