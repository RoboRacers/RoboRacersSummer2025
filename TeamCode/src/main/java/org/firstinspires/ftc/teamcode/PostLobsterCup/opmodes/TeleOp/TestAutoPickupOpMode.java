package org.firstinspires.ftc.teamcode.PostLobsterCup.opmodes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.PostLobsterCup.physical.Intake.Vision.CombinedHSVandAnglePipeline;
import org.firstinspires.ftc.teamcode.PostLobsterCup.coordinator.Robot;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@TeleOp(name = "Test Auto Pickup", group = "Test")
public class TestAutoPickupOpMode extends OpMode {

    OpenCvCamera camera;
    org.firstinspires.ftc.teamcode.PostLobsterCup.physical.Intake.Vision.CombinedHSVandAnglePipeline pipeline;
    CombinedHSVandAnglePipeline.TargetColor targetColor = CombinedHSVandAnglePipeline.TargetColor.BLUE;
    private Robot robot;
    @Override
    public void init() {
        // Set default target color (can be changed if needed)

        int cameraMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance()
                .createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        pipeline = new CombinedHSVandAnglePipeline(targetColor);
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

        robot = new Robot(hardwareMap, telemetry, targetColor);
        robot.setState(Robot.State.INIT);
        try {
            robot.update();
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }

    @Override
    public void init_loop() {


    }
    @Override
    public void start() {

    }

    @Override
    public void loop() {
        try {
            robot.update();
        } catch (InterruptedException e) {
            telemetry.addData("Error", "InterruptedException during update()");
            telemetry.update();
            return;
        }

        // Trigger auto pickup on gamepad1.x once
        if (gamepad1.x) {
            robot.setState(Robot.State.READY_FOR_PICK);
        }
        if(gamepad1.b){
            robot.setState(Robot.State.GRAB_SAMPLE_AND_MOVE_INTAKE_TRANSFER);
        }

        telemetry.addData("Current State", robot.getCurrentState());
        telemetry.addData("Pickup Completed", robot.isPickupCompleted());
        telemetry.update();
    }

    @Override
    public void stop() {
        robot.shutdown();
    }
}
