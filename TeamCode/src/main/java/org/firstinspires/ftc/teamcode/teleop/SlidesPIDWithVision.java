package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.*;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

@TeleOp(name = "GimboVisionSlides", group = "Combined")
public class SlidesPIDWithVision extends OpMode {

    private Follower follower;
    private DcMotor slidesMotor;
    private FtcDashboard dashboard;

    public static double kP = 0.02;
    public static double kI = 0.00;
    public static double kD = 0.0000000001;
    public static double kF = 0.0;

    public double targetSlides = 0.0;
    private double integralSum = 0;
    private double lastError = 0;
    private double maxPower = 0;
    private ElapsedTime timer = new ElapsedTime();
    private final Pose startPose = new Pose(0, 0, 0);

    OpenCvCamera camera;
    CombinedHSVandAnglePipeline pipeline;
    double[][] calibrationData = new double[][]{
            {1140, 401, 13.0, 3.5, 10.0},
            {487, 466, 14.5, -9.8, 7.8},
            {745, 295, 16.2, -5.9, 13.9},
            {1178, 956, 8.5, 2, 1.5}
    };
    PixelToDistanceMapper mapper = new PixelToDistanceMapper(calibrationData);

    public double target(double inches) {
        return (inches * 29);
    }

    enum VisionState {
        IDLE, RETRACTING, WAITING_FOR_RETRACTION, SNAPSHOT_PENDING
    }

    VisionState visionState = VisionState.IDLE;
    private boolean lastX = false;
    private double retractStartTime = 0;

    @Override
    public void init() {
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        slidesMotor = hardwareMap.get(DcMotor.class, "slidesMotor");
        follower.setStartingPose(startPose);
        slidesMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidesMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slidesMotor.setDirection(DcMotorSimple.Direction.REVERSE);

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

        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        pipeline.setTargetColor(CombinedHSVandAnglePipeline.TargetColor.BLUE);
        timer.reset();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        // Handle X press for detection
        boolean detectPressed = gamepad1.x && !lastX;
        lastX = gamepad1.x;

        switch (visionState) {
            case IDLE:
                if (detectPressed) {
                    targetSlides = 0; // retract
                    visionState = VisionState.RETRACTING;
                    retractStartTime = getRuntime();
                }
                break;

            case RETRACTING:
                // Wait until slides close enough to 0 inches
                if (Math.abs(slidesMotor.getCurrentPosition()) < target(0.5) || getRuntime() - retractStartTime > 1.5) {
                    slidesMotor.setPower(0);
                    visionState = VisionState.SNAPSHOT_PENDING;
                    pipeline.triggerSnapshot();
                }
                break;

            case SNAPSHOT_PENDING:
                if (pipeline.hasProcessedSnapshot()) {
                    if (pipeline.getCenter() != null) {
                        PixelToDistanceMapper.DistanceResult result = mapper.getDistanceFromPixel(
                                pipeline.getCenter().x, pipeline.getCenter().y
                        );
                        targetSlides = Math.max(0, Math.min(17, result.forwardDist));
                        telemetry.addData("Detected", pipeline.getDetectedObjectsCount());
                        telemetry.addData("Forward Dist", result.forwardDist);
                    } else {
                        telemetry.addLine("No object detected.");
                        targetSlides = 0;
                    }
                    visionState = VisionState.IDLE;
                }
                break;

            default:
                break;
        }

        // PID Slide Control
        double currentAngle = slidesMotor.getCurrentPosition();
        double error = target(targetSlides) - currentAngle;
        integralSum += error * timer.seconds();
        double derivative = (error - lastError) / timer.seconds();
        double motorPower = (kP * error) + (kI * integralSum) + (kD * derivative);
        motorPower = Math.max(-1.0, Math.min(1.0, motorPower));
        if (Math.abs(motorPower) > Math.abs(maxPower)) {
            maxPower = motorPower;
        }
        slidesMotor.setPower(motorPower);
        lastError = error;
        timer.reset();

        // Drive Control
        follower.setTeleOpMovementVectors(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                true
        );
        follower.update();

        // Telemetry
        telemetry.addData("Target Slide (in)", targetSlides);
        telemetry.addData("Encoder Target", target(targetSlides));
        telemetry.addData("Current Slide Pos", currentAngle);
        telemetry.addData("State", visionState);
        telemetry.addData("Motor Power", motorPower);
        telemetry.addData("Max Power", maxPower);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }

    @Override
    public void stop() {
        camera.stopStreaming();
    }
}
