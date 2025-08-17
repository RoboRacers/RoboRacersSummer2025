package org.firstinspires.ftc.teamcode.teleop.LobsterCup;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.util.Constants;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.*;

import org.firstinspires.ftc.teamcode.teleop.CombinedHSVandAnglePipeline;
import org.firstinspires.ftc.teamcode.PostLobsterCup.physical.Intake.Vision.PixelToDistanceMapper;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

@Autonomous(name = "Auton + Vision for my sigma Ishaan ~ Mith", group = "Main")
public class AutonTest extends OpMode {

    private Follower follower;
    private DcMotor slidesMotor;
    private ElapsedTime timer = new ElapsedTime();

    // PID constants and variables
    public static double kP = 0.02;
    public static double kI = 0.0000000001;
    public static double kD = 0.0000000001;
    public double integralSum = 0;
    public double lastError = 0;
    public double maxPower = 0;

    // Camera and Vision
    private OpenCvCamera camera;
    private CombinedHSVandAnglePipeline pipeline;
    private PixelToDistanceMapper mapper;

    // Pose & Path
    private final Pose startPose = new Pose(0, 0, 0);
    private final Pose subPose = new Pose(20, 10, 0);
    private final Pose basketPose = new Pose(50, 30, Math.toRadians(90));
    private Path toSub, toBasket;

    // FSM 1 – Path Sequence
    private int pathState = 0;

    // FSM 2 – Vision & Slides
    private enum VisionState {
        IDLE, RETRACTING, SNAPSHOT_PENDING, EXTENDING, COMPLETE
    }

    private VisionState visionState = VisionState.IDLE;

    private double targetSlideInches = 0;
    private double retractStartTime;

    public double toTicks(double inches) {
        return inches * 28.229;
    }

    @Override
    public void init() {
        Constants.setConstants(FConstants.class, LConstants.class);

        // Setup camera
        int id = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), id);
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
        pipeline.setTargetColor(CombinedHSVandAnglePipeline.TargetColor.BLUE);
        mapper = new PixelToDistanceMapper(new double[][]{
                {1140, 401, 13.0, 3.5, 10.0},
                {487, 466, 14.5, -9.8, 7.8},
                {745, 295, 16.2, -5.9, 13.9},
                {1178, 956, 8.5, 2, 1.5}
        });

        // Slides
        slidesMotor = hardwareMap.get(DcMotor.class, "slidesMotor");
        slidesMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidesMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slidesMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Follower
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);

        // Paths
        toSub = new Path(new BezierLine(startPose, subPose));
        toSub.setLinearHeadingInterpolation(startPose.getHeading(), subPose.getHeading());

        toBasket = new Path(new BezierLine(subPose, basketPose));
        toBasket.setLinearHeadingInterpolation(subPose.getHeading(), basketPose.getHeading());
    }

    @Override
    public void start() {
        follower.followPath(toSub);
        pathState = 0;
        visionState = VisionState.IDLE;
        timer.reset();
    }

    @Override
    public void loop() {
        follower.update();

        // FSM 1 – Path control
        switch (pathState) {
            case 0:
                if (!follower.isBusy()) {
                    visionState = VisionState.RETRACTING;
                    retractStartTime = getRuntime();
                    pathState = 1;
                }
                break;

            case 1:
                if (visionState == VisionState.COMPLETE) {
                    follower.followPath(toBasket);
                    pathState = 2;
                }
                break;

            case 2:
                if (!follower.isBusy()) {
                    pathState = 3;
                }
                break;
        }

        // FSM 2 – Vision + Slide control
        switch (visionState) {
            case RETRACTING:
                slidesMotor.setPower(-0.4);
                if (Math.abs(slidesMotor.getCurrentPosition()) < toTicks(0.5) || getRuntime() - retractStartTime > 1.5) {
                    slidesMotor.setPower(0);
                    pipeline.triggerSnapshot();
                    visionState = VisionState.SNAPSHOT_PENDING;
                }
                break;

            case SNAPSHOT_PENDING:
                if (pipeline.hasProcessedSnapshot()) {
                    if (pipeline.getCenter() != null) {
                        PixelToDistanceMapper.DistanceResult result = mapper.getDistanceFromPixel(
                                pipeline.getCenter().x, pipeline.getCenter().y
                        );
                        targetSlideInches = Math.max(0, Math.min(18.5, result.forwardDist));
                    } else {
                        targetSlideInches = 0;
                    }
                    visionState = VisionState.EXTENDING;
                    timer.reset();
                }
                break;

            case EXTENDING:
                double error = toTicks(targetSlideInches) - slidesMotor.getCurrentPosition();
                while (Math.abs(error) > 5) {
                    update();
                    error = toTicks(targetSlideInches) - slidesMotor.getCurrentPosition();
                }
                slidesMotor.setPower(0);
                visionState = VisionState.COMPLETE;
                break;

            case COMPLETE:
            case IDLE:
                slidesMotor.setPower(0);
                break;
        }

        // Telemetry
        telemetry.addLine("== Path FSM ==");
        telemetry.addData("Path State", pathState);
        telemetry.addLine("== Vision FSM ==");
        telemetry.addData("Vision State", visionState);
        telemetry.addData("Slide Target (in)", targetSlideInches);
        telemetry.addData("Encoder Target", toTicks(targetSlideInches));
        telemetry.addData("Current Slide Pos", slidesMotor.getCurrentPosition());
        telemetry.addData("Max Power", maxPower);
        telemetry.addLine("== Robot Pose ==");
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }

    // PID update method
    public void update() {
        double targetTicks = toTicks(targetSlideInches);
        double currentPos = slidesMotor.getCurrentPosition();
        double error = targetTicks - currentPos;
        integralSum += error * timer.seconds();
        double derivative = (error - lastError) / timer.seconds();

        double motorPower = (kP * error) + (kI * integralSum) + (kD * derivative);
        motorPower = Math.max(-1.0, Math.min(1.0, motorPower));
        slidesMotor.setPower(motorPower);

        if (Math.abs(motorPower) > Math.abs(maxPower)) {
            maxPower = motorPower;
        }

        lastError = error;
        timer.reset();
    }

    @Override
    public void stop() {
        camera.stopStreaming();
    }
}
