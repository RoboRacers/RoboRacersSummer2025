package org.firstinspires.ftc.teamcode.teleop;


import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class ContSampleDetectionPipeline extends OpenCvPipeline {
    private double targetAngle = 0;

    public Point center;
    private int detectedObjectsCount = 0;
    public double distance;
    public double width;
    public double height;
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

    public double angle;
    private double lastArea = 0;
    public double[] hsvPixel;

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, hsvFrame, Imgproc.COLOR_RGB2HSV);

        // Create masks for each color
        Mat yellowMask = new Mat();
        Mat blueMask = new Mat();
        Mat lowerRedMask = new Mat();
        Mat upperRedMask = new Mat();
        Mat redMask = new Mat();
        Mat combinedMask1 = new Mat();

        Core.inRange(hsvFrame, lowerYellow, upperYellow, yellowMask);
        Core.inRange(hsvFrame, lowerBlue, upperBlue, blueMask);
        Core.inRange(hsvFrame, lowerRed1, upperRed1, lowerRedMask);
        Core.inRange(hsvFrame, lowerRed2, upperRed2, upperRedMask);
        Core.bitwise_or(lowerRedMask, upperRedMask, redMask);
        Core.bitwise_or(yellowMask, blueMask, combinedMask1);
        Core.bitwise_or(combinedMask1, redMask, mask);

        // Morphological filtering
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
                // Approximate contour to polygon
                MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());
                MatOfPoint2f approxCurve = new MatOfPoint2f();
                double epsilon = 0.02 * Imgproc.arcLength(contour2f, true);
                Imgproc.approxPolyDP(contour2f, approxCurve, epsilon, true);

                int vertexCount = approxCurve.toArray().length;

                // Reject shapes with more than 6 sides
                if (vertexCount <= 6) {
                    detectedObjectsCount++;
                    if (area > largestArea) {
                        largestArea = area;
                        largestContour = contour;
                        width = contour.width();
                        height = contour.height();
                    }

                    // Optional: Draw side count
                    Point[] approxPoints = approxCurve.toArray();
                    if (approxPoints.length > 0) {
                        Imgproc.putText(input, vertexCount + " sides",
                                approxPoints[0], Imgproc.FONT_HERSHEY_SIMPLEX,
                                0.5, new Scalar(255, 255, 255), 1);
                    }

                }
            }
        }

        lastArea = largestArea;

        if (largestContour != null) {
            RotatedRect rotatedRect = Imgproc.minAreaRect(new MatOfPoint2f(largestContour.toArray()));
            Point[] points = new Point[4];
            rotatedRect.points(points);

            for (int i = 0; i < 4; i++) {
                Imgproc.line(input, points[i], points[(i + 1) % 4], new Scalar(70, 50, 50), 2);
            }

            center = rotatedRect.center;
            Imgproc.circle(input, center, 5, new Scalar(255, 0, 0), -1);

            angle = rotatedRect.angle;
            if (rotatedRect.size.width < rotatedRect.size.height) {
                angle += 90;
            }

            double frameCenterX = input.width() / 2.0;
            targetAngle = Math.atan2(center.x - frameCenterX, input.height());

            distance = Math.max(width,height);


            String angleText = String.format("Angle: %.2f", angle);
            Imgproc.putText(input, angleText, new Point(center.x - 50, center.y - 20),
                    Imgproc.FONT_HERSHEY_SIMPLEX, 0.8, new Scalar(0, 255, 0), 2);
        }

        // Debug HSV at center pixel
        int x = input.width() / 2;
        int y = input.height() / 2;
        hsvPixel = hsvFrame.get(y, x);
        Imgproc.circle(input, new Point(x, y), 5, new Scalar(0, 255, 0), 5);

        return input; // Change to 'input' when ready to view final result
    }

    public double getTargetAngle() {
        return angle;
    }

    public double[] getHSVCenter() {
        return hsvPixel;
    }
    public Point getCenter(){
        return center;
    }

    public double getArea() {
        return lastArea;
    }
    public double getDistance(){
        return distance;
    }
    public int getDetectedObjectsCount() {
        return detectedObjectsCount;
    }
}
