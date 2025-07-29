package org.firstinspires.ftc.teamcode.teleop.LobsterCup;

import static java.lang.Thread.sleep;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
import org.firstinspires.ftc.teamcode.teleop.CombinedHSVandAnglePipeline;
import org.firstinspires.ftc.teamcode.teleop.PixelToDistanceMapper;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class IntakeWithVision {
    public Servo heightServo, rotateServo, turret;

    public Servo clawServo;
    public DcMotor intakeSlide;
    private double heightPos = 0.5, rotatePos = 0.5, wristPos = 0.5;
    private int targetInches = 0;
    private double kP = 0.02, kI = 0.0000000001, kD = 0.0000000001;
    private double integralSum = 0, lastError = 0;
    private long lastTime = System.nanoTime();

    // Vision-related members
    public Follower follower;
    private OpenCvCamera camera;
    private CombinedHSVandAnglePipeline pipeline;
    private PixelToDistanceMapper mapper;
    private final Pose startPose = new Pose(0, 0, 0);
    private Pose capturedPose = new Pose(0, 0, 0);
    private double centerXpos = 0, centerYpos = 0, varForwardDistance = 0;
    private double targetAngle = 0.0;
    private double manual = 0;
    private FtcDashboard dashboard;
    private ElapsedTime timer = new ElapsedTime();

    enum VisionState {
        IDLE, RETRACTING, SNAPSHOT_PENDING
    }

    enum TransferState {
        IDLE, LIFT, RETRACTING, ROTATE
    }


    private VisionState visionState = VisionState.IDLE;
    private TransferState transferState = TransferState.IDLE;
    Telemetry telemetry;
    private double retractStartTime = 0;


    public void init(HardwareMap hardwareMap) {
        // Hardware
        heightServo = hardwareMap.get(Servo.class, "heightServo");
        rotateServo = hardwareMap.get(Servo.class, "rotateServo");
        clawServo = hardwareMap.get(Servo.class, "clawServo");
        turret = hardwareMap.get(Servo.class, "turret");
        intakeSlide = hardwareMap.get(DcMotor.class, "intakeSlide");

        heightServo.setPosition(heightPos);
        rotateServo.setPosition(rotatePos);
        clawServo.setPosition(0.3);

        intakeSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeSlide.setDirection(DcMotorSimple.Direction.REVERSE);
//
//        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
//        follower.setStartingPose(startPose);





//        pipeline.setTargetColor(CombinedHSVandAnglePipeline.TargetColor.BLUE);
        timer.reset();
    }



    public void runVisionLogic(boolean detectPressed) {

    }


    public void updateTransferState(boolean trigger) {
        switch (transferState) {
            case IDLE:
                if (trigger) {
                    // Start transfer sequence
                    clawServo.setPosition(0.896);
                    transferState = TransferState.LIFT;
                }
                break;

            case LIFT:

                timer.reset();
                while (timer.seconds() < 0.2){

                }
                // Raise intake to transfer height
                setHeightPosition(0.58); // Up for transfer
                // Lift claw slightly (optional for clearance)

                transferState = TransferState.RETRACTING;
                break;

            case RETRACTING:
                // Pull back or rotate out of submersible
                setSlidesTargetInches(-260); // Target transfer height
                setRotatePosition(0.57); // Example: move out of transfer angle
                transferState = TransferState.ROTATE;
                break;

            case ROTATE:
                // Rotate or drop to final orientation
                setTurret(0.92); // Rotate for transfer
                // Optionally open claw
//                clawServo.setPosition(1.0); // Open to release
                transferState = TransferState.IDLE;
                break;
        }
    }





    public void setSlidesTargetInches(double inches) {

        intakeSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeSlide.setDirection(DcMotorSimple.Direction.REVERSE);

        targetInches = (int)inches;
        intakeSlide.setTargetPosition(targetInches);

        intakeSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intakeSlide.setPower(1);

    }

    public void setHeightPosition(double position) {
        heightPos = clamp(position);
        heightServo.setPosition(heightPos);
    }

    public void setRotatePosition(double position) {
        rotatePos = clamp(position);
        rotateServo.setPosition(rotatePos);
    }

    public void setTurret(double position) {
        wristPos = clamp(position);
        turret.setPosition(wristPos);
    }

    public void setClawOpen(boolean open) {
        clawServo.setPosition(open ? 1.0 : 0.3);
    }



    private double clamp(double value) {
        return Math.max(0.0, Math.min(1.0, value));
    }

    public double inchesToTicks(double inches) {
        return inches * 28.51;
    }

    public void telemetry(Telemetry telemetry) {

    }


    public void update() {
        // Slide PID update
//        double ticks = targetInches;
        if (visionState != VisionState.IDLE){
            runVisionLogic(true);
        }
        if (transferState != TransferState.IDLE){
            updateTransferState(true);
        }



//        double current = intakeSlide.getCurrentPosition();
//        double error = ticks - current;
//        double dt = (System.nanoTime() - lastTime) / 1e9;
//        integralSum += error * dt;
//        double derivative = (error - lastError) / dt;
//        double power = kP * error + kI * integralSum + kD * derivative;
//        power = Math.max(-1, Math.min(1, power));
//        intakeSlide.setPower(power);
//        lastError = error;
//        lastTime = System.nanoTime();


    }

    public void stop() {
        camera.stopStreaming();
    }
}
