package org.firstinspires.ftc.teamcode.PostLobsterCup.Layer1.Intake.Vision;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class CombinedHSVandAnglePipeline extends OpenCvPipeline {
    // Enum to define the target color for detection
    public enum TargetColor { RED, BLUE, YELLOW }
    private TargetColor targetColor; // Default target color

    //make constructor public so it can be instantiated in the robot class
    public CombinedHSVandAnglePipeline(TargetColor targetColor) {
        // Constructor can be used to initialize any resources if needed
        this.targetColor = targetColor;
    }

    // Flags to control processing
    private boolean shouldProcess = false; // Flag to indicate if a frame should be processed
    private boolean hasProcessed = false; // Flag to indicate if a frame has been processed

    // Center point of the detected object
    public Point center;

    // HSV color ranges for yellow detection
    private final Scalar lowerYellow = new Scalar(20, 100, 100);
    private final Scalar upperYellow = new Scalar(30, 255, 255);

    // HSV color ranges for blue detection
    private final Scalar lowerBlue = new Scalar(100, 150, 50);
    private final Scalar upperBlue = new Scalar(140, 255, 255);

    // HSV color ranges for red detection (split into two ranges due to HSV wrap-around)
    private final Scalar lowerRed1 = new Scalar(0, 120, 70);
    private final Scalar upperRed1 = new Scalar(10, 255, 255);
    private final Scalar lowerRed2 = new Scalar(170, 120, 70);
    private final Scalar upperRed2 = new Scalar(180, 255, 255);

    // Mats for image processing
    private final Mat hsvFrame = new Mat(); // Mat to store the HSV converted frame
    private final Mat mask = new Mat(); // Mat to store the color mask
    private final Mat processedMask = new Mat(); // Mat to store the morphologically processed mask
    private final Mat hierarchy = new Mat(); // Mat to store contour hierarchy information

    // List to store detected contours
    private final List<MatOfPoint> contours = new ArrayList<>();

    // Minimum contour area to be considered a valid object
    private static final double MIN_CONTOUR_AREA = 500.0;
    // Public variables to store detection results
    public double distance; // Placeholder for distance calculation, currently uses max of width/height
    public double width; // Width of the detected object's bounding box
    public double height; // Height of the detected object's bounding box
    public double angle; // Angle of the detected object's minimum area rectangle
    private double lastArea = 0; // Area of the largest detected contour in the last processed frame
    private double targetAngle = 0; // Angle from the center of the frame to the detected object
    private int detectedObjectsCount = 0; // Number of objects detected in the current frame
    private double[] hsvPixel; // HSV values of the center pixel of the frame (for debugging)

    /**
     * Sets the target color for detection.
     * @param color The TargetColor enum value (RED, BLUE, YELLOW).
     */
    public void setTargetColor(TargetColor color) {
        this.targetColor = color;
    }

    /**
     * Triggers the processing of the next frame.
     * Call this method when you want the pipeline to process a snapshot.
     */
    public void triggerSnapshot() {
        shouldProcess = true;
        hasProcessed = false;
    }

    /**
     * Checks if a snapshot has been processed since the last trigger.
     * @return True if a snapshot has been processed, false otherwise.
     */
    public boolean hasProcessedSnapshot() {
        return hasProcessed;
    }

    /**
     * Processes the input frame from the camera.
     * This method is called by the EasyOpenCV library for each frame.
     * @param input The input frame in RGB format.
     * @return The processed frame, with detections drawn on it if any.
     */
    @Override
    public Mat processFrame(Mat input) {
        // If shouldProcess is false, return the input frame without processing
        if (!shouldProcess) return input;

        // Convert the input frame from RGB to HSV color space
        Imgproc.cvtColor(input, hsvFrame, Imgproc.COLOR_RGB2HSV);

        // Create a mask based on the target color
        Mat mask1 = new Mat(), mask2 = new Mat(); // Temporary masks for red color detection

        switch (targetColor) {
            case YELLOW:
                Core.inRange(hsvFrame, lowerYellow, upperYellow, mask);
                break;
            case BLUE:
                Core.inRange(hsvFrame, lowerBlue, upperBlue, mask);
                break;
            case RED:
                // For red, combine two masks due to HSV color space wrapping around
                Core.inRange(hsvFrame, lowerRed1, upperRed1, mask1);
                Core.inRange(hsvFrame, lowerRed2, upperRed2, mask2);
                Core.bitwise_or(mask1, mask2, mask);
                break;
        }

        // Apply morphological operations to reduce noise and enhance the detected regions
        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
        Imgproc.morphologyEx(mask, processedMask, Imgproc.MORPH_OPEN, kernel); // MORPH_OPEN removes small noise

        // Find contours in the processed mask
        contours.clear(); // Clear previous contours
        Imgproc.findContours(processedMask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        detectedObjectsCount = 0; // Reset detected objects count for the current frame
        MatOfPoint largestContour = null;
        double largestArea = 0;

        // Iterate through all found contours
        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            // Filter contours based on minimum area
            if (area > MIN_CONTOUR_AREA) {
                // Approximate the contour shape to a polygon
                MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());
                MatOfPoint2f approxCurve = new MatOfPoint2f();
                double epsilon = 0.02 * Imgproc.arcLength(contour2f, true);
                Imgproc.approxPolyDP(contour2f, approxCurve, epsilon, true);

                // Filter contours based on the number of vertices (e.g., to find shapes like hexagons or less)
                int vertexCount = approxCurve.toArray().length;
                if (vertexCount <= 6) { // Example: Filter for objects with 6 or fewer vertices
                    detectedObjectsCount++;
                    // If this contour is larger than the previous largest, update it
                    if (area > largestArea) {
                        largestArea = area;
                        largestContour = contour;
                        width = contour.width(); // Get the width of the bounding box of the contour
                        height = contour.height(); // Get the height of the bounding box of the contour
                    }
                }
            }
        }

        lastArea = largestArea; // Store the area of the largest detected object

        // If a largest contour is found
        if (largestContour != null) {
            // Get the minimum area rotated rectangle enclosing the contour
            RotatedRect rect = Imgproc.minAreaRect(new MatOfPoint2f(largestContour.toArray()));
            Point[] points = new Point[4];
            rect.points(points); // Get the vertices of the rectangle
            // Draw the rotated rectangle on the input frame
            for (int i = 0; i < 4; i++) {
                Imgproc.line(input, points[i], points[(i + 1) % 4], new Scalar(70, 50, 50), 2);
            }

            center = rect.center; // Get the center of the rotated rectangle
            // Draw a circle at the center of the detected object
            Imgproc.circle(input, center, 5, new Scalar(255, 0, 0), -1);

            angle = rect.angle; // Get the angle of the rotated rectangle
            // Adjust the angle to be consistent (e.g., always between 0 and 180, or -90 to 90)
            if (rect.size.width < rect.size.height) {
                angle += 90; // If width < height, the angle is along the shorter side, so add 90
            }

            // Calculate the angle from the center of the frame to the detected object's center
            double frameCenterX = input.width() / 2.0;
            targetAngle = Math.atan2(center.x - frameCenterX, input.height()); // atan2 provides angle in radians

            // Display the angle of the object on the frame
            String angleText = String.format("Angle: %.2f", angle);
            Imgproc.putText(input, angleText, new Point(center.x - 50, center.y - 20),
                    Imgproc.FONT_HERSHEY_SIMPLEX, 0.8, new Scalar(0, 255, 0), 2);

            // Placeholder for distance: uses the maximum of the bounding box width or height
            // This is a very rough estimate and should be replaced with a proper distance calculation
            // based on known object size and focal length if accurate distance is needed.
            distance = Math.max(width,height);
        }

        // Debugging: Get HSV values of the center pixel of the frame
        int cx = input.width() / 2;
        int cy = input.height() / 2;
        hsvPixel = hsvFrame.get(cy, cx); // Get HSV values at the center
        // Draw a circle at the center of the frame for visual debugging
        Imgproc.circle(input, new Point(cx, cy), 5, new Scalar(0, 255, 0), 5);

        // Mark that processing is complete for this snapshot
        hasProcessed = true;
        shouldProcess = false; // Reset the trigger

        // Release Mats to free memory
        mask1.release();
        mask2.release();
        // processedMask.release(); // Releasing this here might cause issues if it's used elsewhere or if processFrame is called rapidly.
        // Consider if it needs to be a member variable if it's reused.
        // hierarchy.release();   // Same as above for hierarchy.
        // hsvFrame.release();    // Same as above for hsvFrame.
        // contours.clear();      // Already cleared at the beginning of contour processing.
        kernel.release();

        return input; // Return the processed frame
    }

    /**
     * Gets the angle of the detected object's minimum area rectangle.
     * @return The angle in degrees.
     */
    public double getTargetAngle() { return angle; }

    /**
     * Gets the "distance" metric (currently max of detected object's width/height).
     * Replace with actual distance calculation if needed.
     * @return The distance value.
     */
    public double getDistance(){
        return distance;
    }

    /**
     * Gets the HSV values of the pixel at the center of the frame.
     * @return A double array containing H, S, and V values.
     */
    public double[] getHSVCenter() { return hsvPixel; }

    /**
     * Gets the area of the largest detected contour.
     * @return The area in pixels.
     */
    public double getArea() { return lastArea; }

    /**
     * Gets the number of objects detected in the last processed frame
     * that met the area and vertex count criteria.
     * @return The count of detected objects.
     */
    public int getDetectedObjectsCount() { return detectedObjectsCount; }

    /**
     * Gets the center point of the largest detected object.
     * @return The center point (x, y) or null if no object was detected.
     */
    public Point getCenter(){
        return center;
    }
}
