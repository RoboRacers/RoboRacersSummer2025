package org.firstinspires.ftc.teamcode.teleop;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class CombinedHSVandAnglePipeline extends OpenCvPipeline {
    public enum TargetColor { RED, BLUE, YELLOW }
    private TargetColor targetColor = TargetColor.BLUE;

    private boolean shouldProcess = false;
    private boolean hasProcessed = false;

    public Point center;

    private final Scalar lowerYellow = new Scalar(20, 100, 100);
    private final Scalar upperYellow = new Scalar(30, 255, 255);

    private final Scalar lowerBlue = new Scalar(100, 150, 50);
    private final Scalar upperBlue = new Scalar(140, 255, 255);

    private final Scalar lowerRed1 = new Scalar(0, 120, 70);
    private final Scalar upperRed1 = new Scalar(10, 255, 255);

    private final Scalar lowerRed2 = new Scalar(170, 120, 70);
    private final Scalar upperRed2 = new Scalar(180, 255, 255);

    private final Mat hsvFrame = new Mat();
    private final Mat mask = new Mat();
    private final Mat processedMask = new Mat();
    private final Mat hierarchy = new Mat();

    private final List<MatOfPoint> contours = new ArrayList<>();

    private static final double MIN_CONTOUR_AREA = 500.0;
    public double distance;
    public double width;
    public double height;
    public double angle;
    private double lastArea = 0;
    private double targetAngle = 0;
    private int detectedObjectsCount = 0;
    private double[] hsvPixel;

    public void setTargetColor(TargetColor color) {
        this.targetColor = color;
    }

    public void triggerSnapshot() {
        shouldProcess = true;
        hasProcessed = false;
    }

    public boolean hasProcessedSnapshot() {
        return hasProcessed;
    }

    @Override
    public Mat processFrame(Mat input) {
        if (!shouldProcess) return input;

        Imgproc.cvtColor(input, hsvFrame, Imgproc.COLOR_RGB2HSV);

        // Create target mask
        Mat mask1 = new Mat(), mask2 = new Mat();

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

        // Find contours
        contours.clear();
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

                int vertexCount = approxCurve.toArray().length;
                if (vertexCount <= 6) {
                    detectedObjectsCount++;
                    if (area > largestArea) {
                        largestArea = area;
                        largestContour = contour;
                        width = contour.width();
                        height = contour.height();
                    }
                }
            }
        }

        lastArea = largestArea;

        if (largestContour != null) {
            RotatedRect rect = Imgproc.minAreaRect(new MatOfPoint2f(largestContour.toArray()));
            Point[] points = new Point[4];
            rect.points(points);
            for (int i = 0; i < 4; i++) {
                Imgproc.line(input, points[i], points[(i + 1) % 4], new Scalar(70, 50, 50), 2);
            }

            center = rect.center;
            Imgproc.circle(input, center, 5, new Scalar(255, 0, 0), -1);

            angle = rect.angle;
            if (rect.size.width < rect.size.height) {
                angle += 90;
            }

            double frameCenterX = input.width() / 2.0;
            targetAngle = Math.atan2(center.x - frameCenterX, input.height());

            String angleText = String.format("Angle: %.2f", angle);
            Imgproc.putText(input, angleText, new Point(center.x - 50, center.y - 20),
                    Imgproc.FONT_HERSHEY_SIMPLEX, 0.8, new Scalar(0, 255, 0), 2);

            distance = Math.max(width,height);
        }

        // Debug center HSV pixel
        int cx = input.width() / 2;
        int cy = input.height() / 2;
        hsvPixel = hsvFrame.get(cy, cx);
        Imgproc.circle(input, new Point(cx, cy), 5, new Scalar(0, 255, 0), 5);

        hasProcessed = true;
        shouldProcess = false;
        return input;
    }

    public double getTargetAngle() { return angle; }

    public double getDistance(){
        return distance;
    }
    public double[] getHSVCenter() { return hsvPixel; }
    public double getArea() { return lastArea; }
    public int getDetectedObjectsCount() { return detectedObjectsCount; }

    public Point getCenter(){
        return center;
    }
}
