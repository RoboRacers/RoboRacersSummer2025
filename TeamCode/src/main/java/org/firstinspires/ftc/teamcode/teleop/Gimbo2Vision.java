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

@TeleOp(name = "Gimbo2Vision", group = "Combined")
public class Gimbo2Vision extends OpMode {

    private Follower follower;
    private DcMotor slidesMotor;
    private FtcDashboard dashboard;

    public static double kP = 0.08;
    public static double kI = 0.00;
    public static double kD = 0.0001;

    private double integralSum = 0;
    private double lastError = 0;
    private double maxPower = 0;
    private ElapsedTime timer = new ElapsedTime();
    private final Pose startPose = new Pose(0,0,0);

    private double targetAngle = 0.0;       // Live target (adjusted for movement)
    private double initTarget = 0.0;        // Vision-set initial target

    public Pose currentPose;

    // Vision
    OpenCvCamera camera;
    CombinedHSVandAnglePipeline pipeline;
    PixelToDistanceMapper mapper;

    public double target(double inches) {
        return inches * 30.2439;
    }

    @Override
    public void init() {
        // Slides + Follower setup
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        slidesMotor = hardwareMap.get(DcMotor.class, "slidesMotor");
        follower.setStartingPose(startPose);
        slidesMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidesMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slidesMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Vision setup
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
            }
        });

        double[][] calibrationData = new double[][]{
                {1140, 401, 13.0, 3.5, 10.0},
                {487, 466, 14.5, -9.8, 7.8},
                {745, 295, 16.2, -5.9, 13.9}
        };
        mapper = new PixelToDistanceMapper(calibrationData);

        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        timer.reset();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
        initTarget = 15;  // Default initial target (no vision yet)
    }

    @Override
    public void loop() {
        // Vision Processing
        pipeline.setTargetColor(CombinedHSVandAnglePipeline.TargetColor.RED);
        if (gamepad1.x) {
            pipeline.triggerSnapshot();
            currentPose = follower.getPose();
        }

        if (pipeline.hasProcessedSnapshot()) {
            PixelToDistanceMapper.DistanceResult result = mapper.getDistanceFromPixel(
                    pipeline.getCenter().x, pipeline.getCenter().y
            );
            initTarget = Math.max(0, Math.min(17, result.forwardDist));
        }

        // Field-Relative Slide Adjustment
        targetAngle = initTarget - follower.getPose().getX() - currentPose.getX();
        targetAngle = Math.max(0, Math.min(17, targetAngle));

        // PID Control for Slides
        double currentPos = slidesMotor.getCurrentPosition();
        double error = target(targetAngle) - currentPos;
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

        // Movement Control
        follower.setTeleOpMovementVectors(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                true
        );
        follower.update();

        // Telemetry
        telemetry.addData("Detected Objects", pipeline.getDetectedObjectsCount());
        telemetry.addData("Vision Target (Init)", initTarget);
        telemetry.addData("Adjusted Target (With Movement)", targetAngle);
        telemetry.addData("Encoder Target", target(targetAngle));
        telemetry.addData("Current Pos", currentPos);
        telemetry.addData("Error", error);
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
