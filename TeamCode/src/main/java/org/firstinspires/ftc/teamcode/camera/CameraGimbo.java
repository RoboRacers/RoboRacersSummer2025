package org.firstinspires.ftc.teamcode.camera;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

@Config
@Disabled
@TeleOp(name = "Camera GImbo", group = "Concept")
public class CameraGimbo extends LinearOpMode {
    private OpenCvCamera camera;
    private BlueObjectDetectionPipeline pipeline;

    private DcMotor slidesMotor;

    public Pose currentPose;
    //    private AnalogInput potentiometer;
    private FtcDashboard dashboard;

    double motorPower;
    double maxPower;

    public static double kP = 0.011;
    public static double kI = 0.00;
    public static double kD = 0.0000000000000;
    public static double kF = 0.0;
    public static double targetAngle = 0.0; // Target angle in degrees
    public double initTarget = 0.0; // Target angle in degrees
    public double sampleOffset=0;
    private double integralSum = 0;
    private double lastError = 0;
    private double lastTarget = 0;

    public double poseErrorX = 0;

    public double poseErrorY = 0;
    private ElapsedTime timer = new ElapsedTime();

    public static final double CAMERA_TILT = 31.0;
    private Follower follower;
    private final Pose startPose = new Pose(0, 0, 0);

    public double target(double inches){
        return (inches * 27.90697);
    }
    public double targetReverse(double cm){
        return (cm / 27.90697);
    }



    @Override
    public void runOpMode() {
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);
        follower.startTeleopDrive();

        int camMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        camera = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), camMonitorViewId);

        pipeline = new BlueObjectDetectionPipeline();
        camera.setPipeline(pipeline);

        telemetry.addLine("Opening camera...");
        telemetry.update();

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
                FtcDashboard.getInstance().startCameraStream(camera, 30);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Error", errorCode);
                telemetry.update();
            }
        });

        while (opModeInInit()){
            follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
            slidesMotor = hardwareMap.get(DcMotor.class, "slidesMotor");
            follower.setStartingPose(startPose);
            slidesMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slidesMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            slidesMotor.setDirection(DcMotorSimple.Direction.REVERSE);

            currentPose = follower.getPose();

            // Initialize FTC Dashboard
            dashboard = FtcDashboard.getInstance();
            telemetry = new MultipleTelemetry(telemetry,FtcDashboard.getInstance().getTelemetry());
            timer.reset();
        }

        waitForStart();

        int framesWithoutDetection = 0;
        final int maxLostFrames = 10;
//        final double stopBeforeIn = 15.0;
        final double stopBeforeIn = 8.8;
//        final double toleranceCm = 5.0;

        final double toleranceCM = 2;
        // Camera setup constants
//        double cameraHeightCm = 26.0;
        double verticalFovDeg = 45.0;
        double horizontalFovDeg = 60.0;
//        double cameraOffsetIn = 15.24;
//        double cameraLateralOffsetIn = 0.0; // + is right, - is left

        double cameraHeightIn = 10.25;
        double cameraOffsetIn = 9;
        double cameraLateralOffsetIn = 5.5;


        follower.startTeleopDrive();
        while (opModeIsActive()) {

            if (!pipeline.hasProcessedFrame()) {
                sleep(50);
                continue;
            }

            List<Rect> blueObjects = pipeline.getDetectedRects();
            if (blueObjects.isEmpty()) {
                framesWithoutDetection++;
                telemetry.addData("Frames without detection", framesWithoutDetection);
                telemetry.update();
                continue;
            }

            // === Find closest object ===
            Rect closestRect = null;
            double closestDistance = Double.MAX_VALUE;

            for (Rect r : blueObjects) {
                int centerY = r.y + r.height / 2;
                double yOffset = centerY - 240;
                double angleToTarget = CAMERA_TILT + (yOffset / 480.0) * verticalFovDeg;
                double distanceFromCamera = cameraHeightIn / Math.sin(Math.toRadians(angleToTarget));
                double horizontalDistance = Math.sqrt(Math.pow(distanceFromCamera, 2) - Math.pow(cameraHeightIn, 2));
                double distanceFromRobot = horizontalDistance + cameraOffsetIn - stopBeforeIn;

                if (distanceFromRobot < closestDistance) {
                    closestDistance = distanceFromRobot;
                    closestRect = r;
                }
            }

            if (closestRect == null) continue;

            // === Calculate movement to closest object ===
            double centerX = closestRect.x + pipeline.getSmoothedWidth() / 2;
            double centerY = closestRect.y + pipeline.getSmoothedWidth() / 2;

            double yOffset = centerY - 240;
            double angleToTarget = CAMERA_TILT + (yOffset / 480.0) * verticalFovDeg;
            double distanceFromCamera = cameraHeightIn / Math.sin(Math.toRadians(angleToTarget));
            double horizontalDistance = Math.sqrt(Math.pow(distanceFromCamera, 2) - Math.pow(cameraHeightIn, 2));
            double distanceFromRobot = horizontalDistance + cameraOffsetIn - stopBeforeIn;




            double pixelErrorX = centerX - 320;
            double pixelsPerDegree = 640 / horizontalFovDeg;
            double errorXDegrees = pixelErrorX / pixelsPerDegree;
            double lateralOffsetIn = distanceFromRobot * Math.tan(Math.toRadians(errorXDegrees)) + cameraLateralOffsetIn;

            if (gamepad1.cross) {
                currentPose = follower.getPose();
//                poseErrorX = (distanceFromRobot) - currentPose.getX();
//                poseErrorY = (lateralOffsetCm) - currentPose.getY();
                poseErrorX = distanceFromRobot;
                poseErrorY = lateralOffsetIn;
                timer.reset();
                while (timer.seconds() < 1){
                    sampleOffset = 3;
                }
            }

            if (poseErrorX <= 2){
                sampleOffset = 0;
            }
            else if(gamepad1.square){
                sampleOffset +=1;
            }
            else if (gamepad1.circle){
                sampleOffset -= 1;
            }


            targetAngle = poseErrorX - follower.getPose().getX() + currentPose.getX() - sampleOffset;


            if (targetAngle >= 20.5){
                targetAngle = 20.5;
            }
            else if (targetAngle <= 0.5){
                targetAngle = 0.5;
            }


            double currentAngle = slidesMotor.getCurrentPosition();

            // Calculate error (using angles)
            double error = target(targetAngle) - currentAngle;

            integralSum += error * timer.seconds();

            double derivative = (error - lastError) / timer.seconds();

//            double feedForward = kF * Math.sin(Math.toRadians(currentAngle));

            motorPower = (kP * error) + (kI * integralSum) + (kD * derivative);

            motorPower = Math.max(-1.0, Math.min(1.0, motorPower));

            if(Math.abs(motorPower) > Math.abs(maxPower)){
                maxPower = motorPower;
            }

            slidesMotor.setPower(motorPower);

            lastError = error;
            lastTarget = target(targetAngle);
            timer.reset();




//            double forwardPower = clamp(poseErrorX * 0.05, -0.5, 0.5);
//            double strafePower = clamp(poseErrorY * 0.05, -0.3, 0.3);


            // Uncomment to move
            // follower.setTeleOpMovementVectors(forwardPower, strafePower, 0, true);
            // follower.update();

            follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
            follower.update();

            /* Telemetry Outputs of our Follower */
            telemetry.addData("X", follower.getPose().getX());
            telemetry.addData("Y", follower.getPose().getY());
            telemetry.addData("Heading in Degrees", Math.toDegrees(follower.getPose().getHeading()));

            telemetry.addData("Error Difference between camera and slide target", poseErrorX - targetAngle);



            telemetry.addData("Target Angle", target(targetAngle));
            telemetry.addData("Error PID ticks", error);


            telemetry.addData("Target", targetAngle);
//            telemetry.addData("Current Angle", currentAngle);
            telemetry.addData("Error PID inch", targetReverse(error));

            telemetry.addData("Distance to target (cm)", distanceFromRobot);
            telemetry.addData("Lateral offset (cm)", lateralOffsetIn);
            telemetry.addData("Pose error X", poseErrorX);
            telemetry.addData("Pose error Y", poseErrorY);
//            telemetry.addData("Forward power", forwardPower);
//            telemetry.addData("Strafe power", strafePower);
            telemetry.update();

            sleep(50);
        }

        camera.stopStreaming();
        camera.closeCameraDevice();
    }




    private double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }

    // === Blue Object Detection Pipeline ===
    // === Blue Object Detection Pipeline ===
    private static class BlueObjectDetectionPipeline extends OpenCvPipeline {
        private boolean processed = false;
        private final List<Rect> detectedRects = new ArrayList<>();
        private final Mat hsv = new Mat();
        private final Mat mask = new Mat();
        private final Mat rgbaCopy = new Mat();

        private static final Scalar LOWER_BLUE = new Scalar(100, 150, 50);
        private static final Scalar UPPER_BLUE = new Scalar(140, 255, 255);

        // === New fields for frame averaging ===
        private double smoothedWidth = 0;
        private double smoothedHeight = 0;

        public boolean hasProcessedFrame() {
            return processed;
        }

        public List<Rect> getDetectedRects() {
            return detectedRects;
        }

        public double getSmoothedWidth() {
            return smoothedWidth;
        }

        public double getSmoothedHeight() {
            return smoothedHeight;
        }

        @Override
        public Mat processFrame(Mat input) {
            processed = true;
            input.copyTo(rgbaCopy);

            Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);
            Core.inRange(hsv, LOWER_BLUE, UPPER_BLUE, mask);

            // === Morphological filtering (noise reduction) ===
            Imgproc.erode(mask, mask, Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3)));
            Imgproc.dilate(mask, mask, Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3)));

            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();
            Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            detectedRects.clear();

            double maxArea = 0;
            Rect largestRect = null;

            for (MatOfPoint contour : contours) {
                double area = Imgproc.contourArea(contour);
                if (area > 500) {
                    Rect rect = Imgproc.boundingRect(contour);
                    detectedRects.add(rect);
                    if (area > maxArea) {
                        maxArea = area;
                        largestRect = rect;
                    }
                }
            }

            // === Apply smoothing to the largest rect only ===
            if (largestRect != null) {
                smoothedWidth = 0.8 * smoothedWidth + 0.2 * largestRect.width;
                smoothedHeight = 0.8 * smoothedHeight + 0.2 * largestRect.height;
            }

            // Draw all valid rects
            for (Rect rect : detectedRects) {
                Imgproc.rectangle(input, rect, new Scalar(255, 0, 0), 2);
            }

            return input;
        }
    }

}