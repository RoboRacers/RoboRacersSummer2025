package org.firstinspires.ftc.teamcode.teleop.gimboCode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

@TeleOp(name = "Gimbo Submersible Avoidance", group = "Examples")
public class GimboSubmersibleAvoidance extends OpMode {
    private Follower follower;
    private DcMotor slidesMotor;
    private FtcDashboard dashboard;

    double motorPower;
    double maxPower;

    // PID Configuration
    public static double kP = 0.02;
    public static double kI = 0.00;
    public static double kD = 0.0000000001;
    public static double kF = 0.0;
    public static double targetAngle = 0.0;
    public double initTarget = 0.0;
    private double integralSum = 0;
    private double lastError = 0;
    private double lastTarget = 0;

    private ElapsedTime timer = new ElapsedTime();
    private final Pose startPose = new Pose(-12, 65, Math.toRadians(270));

    public boolean avoidSubmerisble = true;

    public double target(double inches) {
        return inches * 28.229;
    }

    private boolean isNearRectangle(double robotX, double robotY,
                                    double rectXMin, double rectXMax,
                                    double rectYMin, double rectYMax,
                                    double buffer) {
        return robotX >= rectXMin - buffer && robotX <= rectXMax + buffer &&
                robotY >= rectYMin - buffer && robotY <= rectYMax + buffer;
    }


    @Override
    public void init() {
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        slidesMotor = hardwareMap.get(DcMotor.class, "slidesMotor");
        follower.setStartingPose(startPose);
        slidesMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidesMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slidesMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        timer.reset();
        targetAngle = 15;
    }

    @Override
    public void init_loop() {
        double currentAngle = slidesMotor.getCurrentPosition();
        double error = target(targetAngle) - currentAngle;
        integralSum += error * timer.seconds();
        double derivative = (error - lastError) / timer.seconds();
        motorPower = (kP * error) + (kI * integralSum) + (kD * derivative);
        motorPower = Math.max(-1.0, Math.min(1.0, motorPower));
        if (Math.abs(motorPower) > Math.abs(maxPower)) {
            maxPower = motorPower;
        }
        slidesMotor.setPower(motorPower);
        lastError = error;
        lastTarget = target(targetAngle);
        timer.reset();
    }

    @Override
    public void start() {
        initTarget = targetAngle;
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        double robotX = follower.getPose().getX();
        double robotY = follower.getPose().getY();

        targetAngle = initTarget - robotX;
        targetAngle = Math.max(0, Math.min(19, targetAngle));

        double safeDistance = 2.0; // You can tune this
        double slideLength = targetAngle; // in inches (or convert from encoder ticks if needed)
        double heading = follower.getPose().getHeading(); // radians
        double slideTipX = robotX + slideLength * Math.cos(heading);
        double slideTipY = robotY + slideLength * Math.sin(heading);

        telemetry.addData("Slide Tipx", slideTipX);

        telemetry.addData("Slide Tipy", slideTipY);

        boolean nearAnyObstacle =
                isNearRectangle(slideTipX, slideTipY, -24.0, -14.25, 23.0, 25.0, safeDistance) ||
                        isNearRectangle(slideTipX, slideTipY, 14.25, 24.0, 23.0, 25.0, safeDistance)  ||
                        isNearRectangle(slideTipX, slideTipY, -24.0, -14.25, -25.0, -23.0, safeDistance) ||
                        isNearRectangle(slideTipX, slideTipY, 14.25, 24.0, -25.0, -23.0, safeDistance);

        if (avoidSubmerisble && nearAnyObstacle) {
            targetAngle = 2.0;
        } else {
            targetAngle = initTarget;
        }



        double currentAngle = slidesMotor.getCurrentPosition();
        double error = target(targetAngle) - currentAngle;
        integralSum += error * timer.seconds();
        double derivative = (error - lastError) / timer.seconds();
        motorPower = (kP * error) + (kI * integralSum) + (kD * derivative);
        motorPower = Math.max(-1.0, Math.min(1.0, motorPower));
        if (Math.abs(motorPower) > Math.abs(maxPower)) {
            maxPower = motorPower;
        }

        slidesMotor.setPower(motorPower);
        lastError = error;
        lastTarget = target(targetAngle);
        timer.reset();

        telemetry.addData("Target Angle (Ticks)", target(targetAngle));
        telemetry.addData("Target (Inches)", targetAngle);
        telemetry.addData("Error", error);
        telemetry.addData("Motor Power", motorPower);
        telemetry.addData("kP", kP);
        telemetry.addData("kI", kI);
        telemetry.addData("kD", kD);
        telemetry.addData("kF", kF);
        telemetry.addData("Max Power Used", maxPower);

        follower.setTeleOpMovementVectors(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x*0.75,
                true
        );
        follower.update();

        telemetry.addData("X", robotX);
        telemetry.addData("Y", robotY);
        telemetry.addData("Heading in Degrees", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
        dashboard.getTelemetry();
    }

    @Override
    public void stop() {
    }
}
