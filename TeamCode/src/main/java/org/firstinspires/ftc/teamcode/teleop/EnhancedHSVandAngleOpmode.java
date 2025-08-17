package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.PostLobsterCup.physical.Intake.Vision.PixelToDistanceMapper;
import org.openftc.easyopencv.*;
@Disabled
@TeleOp(name = "Snapshot Vision OpMode", group = "Vision")
public class EnhancedHSVandAngleOpmode extends LinearOpMode {
    OpenCvCamera camera;
    CombinedHSVandAnglePipeline pipeline;



    double[][] calibrationData = new double[][]{
            {600, 775, 4,  -13, 11},
            {1254,  788, 5, -1.2,  12},
            {1695,  752, 5, 9.6, 20},
            {740, 478, 12, -11.75, 15},
            {905, 509, 10.5, -9, 14},
            {1110, 523, 11, -3, 15},
            {1278, 531, 10.5, 2.2, 20},
            {1486, 522, 12, 10, 25},
            {800, 348, 20, 9.25, 24},
            {1064, 347, 21.5, -1, 24},
            {1307, 394, 20, 8.5, 30}
    };

    PixelToDistanceMapper mapper = new PixelToDistanceMapper(calibrationData);

    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance()
                .createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        pipeline = new CombinedHSVandAnglePipeline();
        camera.setPipeline(pipeline);

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


        telemetry.addLine("Press A for RED, B for BLUE, Y for YELLOW");
        telemetry.addLine("Press X to SNAPSHOT");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a) {
                pipeline.setTargetColor(CombinedHSVandAnglePipeline.TargetColor.RED);
                telemetry.addLine("Target Color: RED");
            } else if (gamepad1.b) {
                pipeline.setTargetColor(CombinedHSVandAnglePipeline.TargetColor.BLUE);
                telemetry.addLine("Target Color: BLUE");
            } else if (gamepad1.y) {
                pipeline.setTargetColor(CombinedHSVandAnglePipeline.TargetColor.YELLOW);
                telemetry.addLine("Target Color: YELLOW");
            }

            if (gamepad1.x) {
                pipeline.triggerSnapshot();
            }

            if (pipeline.hasProcessedSnapshot()) {
                double angleDeg = pipeline.getTargetAngle();

                telemetry.addData("Center", pipeline.getCenter());

                telemetry.addData("Angle (deg)", angleDeg);
                telemetry.addData("Detected Objects", pipeline.getDetectedObjectsCount());
                telemetry.addData("pixels", pipeline.getDistance());

                PixelToDistanceMapper.DistanceResult result = mapper.getDistanceFromPixel(pipeline.getCenter().x, pipeline.getCenter().y);

                telemetry.addData("Direct Distance", result.directDist);
                telemetry.addData("Forward", result.forwardDist);
                telemetry.addData("Horizontal Offset", result.horizOffset);

                double[] hsv = pipeline.getHSVCenter();
                if (hsv != null) {
                    telemetry.addData("Center HSV", String.format("[%.0f, %.0f, %.0f]", hsv[0], hsv[1], hsv[2]));
                }
            }

            telemetry.update();
            sleep(100);
        }

        camera.stopStreaming();
    }
}
