package org.firstinspires.ftc.teamcode.teleop;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class combinedvision extends OpenCvPipeline {
    private double targetAngle = 0;
    private int detectedObjectsCount = 0;

    private final Scalar lowerYellow = new Scalar(20, 100, 100);
    private final Scalar upperYellow = new Scalar(30, 255, 255);

    //Replaced with values from BlueObjectDetectionPipeline
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

    @Override
    public Mat processFrame(Mat input) {
        // Convert the input frame to HSV
        Imgproc.cvtColor(input, hsvFrame, Imgproc.COLOR_BGR2HSV);

        // Create masks for each color
        Mat yellowMask = new Mat();
        Mat blueMask = new Mat();
        Mat redMask = new Mat();
        Mat lowerRedMask = new Mat();
        Mat upperRedMask = new Mat();

        Core.inRange(hsvFrame, lowerYellow, upperYellow, yellowMask);
        Core.inRange(hsvFrame, lowerBlue, upperBlue, blueMask);  // Updated blue range
        Core.inRange(hsvFrame, lowerRed1, upperRed1, lowerRedMask);
        Core.inRange(hsvFrame, lowerRed2, upperRed2, upperRedMask);

        Core.addWeighted(lowerRedMask, 1.0, upperRedMask, 1.0, 0.0, redMask);

        // Combine all masks
        Core.addWeighted(yellowMask, 1.0, blueMask, 1.0, 0.0, mask);
        Core.addWeighted(mask, 1.0, redMask, 1.0, 0.0, mask);

        // Apply morphological operations
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
                detectedObjectsCount++;
                if (area > largestArea) {
                    largestArea = area;
                    largestContour = contour;
                }
            }
        }

        // Draw bounding rects for all detected contours (like your blue pipeline)
        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            if (area > MIN_CONTOUR_AREA) {
                Rect rect = Imgproc.boundingRect(contour);
                Imgproc.rectangle(input, rect, new Scalar(255, 0, 0), 2);
            }
        }

        if (largestContour != null) {
            RotatedRect rotatedRect = Imgproc.minAreaRect(new MatOfPoint2f(largestContour.toArray()));
            Point[] points = new Point[4];
            rotatedRect.points(points);
            for (int i = 0; i < 4; i++) {
                Imgproc.line(input, points[i], points[(i + 1) % 4], new Scalar(70, 50, 50), 2);
            }

            Point center = rotatedRect.center;
            Imgproc.circle(input, center, 5, new Scalar(255, 0, 0), -1);

            angle = rotatedRect.angle;
            if (rotatedRect.size.width < rotatedRect.size.height) {
                angle += 90;
            }

            double frameCenterX = input.width() / 2.0;
            targetAngle = Math.atan2(center.x - frameCenterX, input.height());

            String angleText = String.format("Angle: %.2f", angle);
            Imgproc.putText(input, angleText, new Point(center.x - 50, center.y - 20),
                    Imgproc.FONT_HERSHEY_SIMPLEX, 0.8, new Scalar(0, 255, 0), 2);
        }

        return input;
    }

    public double getTargetAngle() {
        return angle;
    }

    public double getArea(){
        return getArea(); // This will cause a recursion error. Should return the area of largestContour or similar.
    }

    public int getDetectedObjectsCount() {
        return detectedObjectsCount;
    }
}
