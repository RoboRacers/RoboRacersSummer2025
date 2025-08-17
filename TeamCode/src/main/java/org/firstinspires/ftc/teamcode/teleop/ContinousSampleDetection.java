package org.firstinspires.ftc.teamcode.teleop;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.PostLobsterCup.physical.Intake.Vision.PixelToDistanceMapper;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Disabled
@TeleOp(name = "Enhanced HSV and Angle Detection OpMode", group = "Vision")
public class ContinousSampleDetection extends LinearOpMode {

    private boolean isAveraging = false;
    private int validFrames = 0;
    private final int maxValidFrames = 100;
    private double distanceSum = 0;
    private double averageDistance = 0;
    // Format: new double[] {pixelX, pixelY, directDist, horizOffset, forwardDist}
    double[][] calibrationData = new double[][]{
            {1140, 401, 13.0,  3.5, 10.0},
            {487,  466, 14.5, -9.8,  7.8},
            {745,  295, 16.2, -5.9, 13.9}
    };

    PixelToDistanceMapper mapper = new PixelToDistanceMapper(calibrationData);

    OpenCvCamera camera;
    org.firstinspires.ftc.teamcode.teleop.ContSampleDetectionPipeline pipeline;
    Servo claw;

    @Override
    public void runOpMode() {
        // Initialize the camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        // Initialize the pipeline
        pipeline = new org.firstinspires.ftc.teamcode.teleop.ContSampleDetectionPipeline();
        camera.setPipeline(pipeline);

//        claw = hardwareMap.get(Servo.class, "rotateClaw");

        // Open the camera
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(1920, 1200, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Error", errorCode);
                telemetry.update();
            }
        });

        waitForStart();

        while (opModeIsActive()) {
            // Get target angle and detected objects count
            double targetAngle = pipeline.getTargetAngle();
            int detectedObjects = pipeline.getDetectedObjectsCount();

            targetAngle *= (180/3.14);
            //NewValue = (((OldValue - OldMin) * (NewMax - NewMin)) / (OldMax - OldMin)) + NewMin

            double output = (((pipeline.angle - 0) * (0.9 - 0.1)) / (180 - 0)) + 0.1;

//            claw.setPosition(output);
//            if (gamepad1.a) {
//                pipeline.setTargetColor(CombinedHSVandAnglePipeline.TargetColor.RED);
//                telemetry.addLine("Target Color: RED");
//            } else if (gamepad1.b) {
//                pipeline.setTargetColor(CombinedHSVandAnglePipeline.TargetColor.BLUE);
//                telemetry.addLine("Target Color: BLUE");
//            } else if (gamepad1.y) {
//                pipeline.setTargetColor(CombinedHSVandAnglePipeline.TargetColor.YELLOW);
//                telemetry.addLine("Target Color: YELLOW");
//            }

            // Start averaging
            if (gamepad1.x && !isAveraging) {
                distanceSum = 0;
                validFrames = 0;
                averageDistance = 0;
                isAveraging = true;
                telemetry.addLine("Started 100-frame averaging...");
            }

            // Perform averaging
            if (isAveraging) {
                double dist = pipeline.getDistance();

                if (dist > 0) {
                    distanceSum += dist;
                    validFrames++;
                    telemetry.addData("Valid Snapshots", validFrames + "/" + maxValidFrames);
                } else {
                    telemetry.addLine("Waiting for valid detection...");
                }

                if (validFrames >= maxValidFrames) {
                    averageDistance = distanceSum / maxValidFrames;
                    isAveraging = false;
                    telemetry.addLine("✅ Averaging Complete");
                }
            }

            if (!isAveraging && validFrames >= maxValidFrames) {
                telemetry.addData("Average Pixel Distance", averageDistance);
                telemetry.addData("Center", pipeline.getCenter());
                telemetry.addData("Width", pipeline.width);
                telemetry.addData("Height", pipeline.height);

                PixelToDistanceMapper.DistanceResult result = mapper.getDistanceFromPixel(pipeline.getCenter().x, pipeline.getCenter().y);

                telemetry.addData("Direct Distance", result.directDist);
                telemetry.addData("Forward", result.forwardDist);
                telemetry.addData("Horizontal Offset", result.horizOffset);
                telemetry.update();

            } else if (!isAveraging) {
                telemetry.addData("Live Pixel Distance", pipeline.getDistance());
            }

            // Display telemetry
//            telemetry.addData("Target Angle (Radians)", targetAngle);
//            telemetry.addData("Detected Objects", detectedObjects);
//            telemetry.addData("pixels", pipeline.getDistance());
//            telemetry.addData("Center HSV value", pipeline.getHSVCenter().toString());
            telemetry.update();

            sleep(100); // Reduce CPU usage
        }

        camera.stopStreaming();
    }
}