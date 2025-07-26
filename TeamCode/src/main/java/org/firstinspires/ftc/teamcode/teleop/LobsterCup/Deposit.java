package org.firstinspires.ftc.teamcode.teleop.LobsterCup;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Deposit {
    private DcMotor verticalSlides;
    private Servo liftServoLeft;
    private Servo liftServoRight;
    private Servo wristServo;
    private Servo clawServo;

    private double targetInches = 0;
    private double kP = 0.02, kI = 0.0000000001, kD = 0.0000000001;
    private double integralSum = 0, lastError = 0;
    private long lastTime = System.nanoTime();

    public void init(HardwareMap hardwareMap) {
        verticalSlides = hardwareMap.get(DcMotor.class, "vSlides");
        liftServoLeft  = hardwareMap.get(Servo.class, "liftL");
        liftServoRight = hardwareMap.get(Servo.class, "liftR");
        wristServo     = hardwareMap.get(Servo.class, "wrist");
        clawServo      = hardwareMap.get(Servo.class, "cS"); // Shared with Intake

        verticalSlides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        verticalSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalSlides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setSlidesTargetInches(double inches) {
        targetInches = inches;
    }

    public void update() {
        double ticks = inchesToTicks(targetInches);
        double current = verticalSlides.getCurrentPosition();
        double error = ticks - current;
        double dt = (System.nanoTime() - lastTime) / 1e9;
        integralSum += error * dt;
        double derivative = (error - lastError) / dt;
        double power = kP * error + kI * integralSum + kD * derivative;
        power = Math.max(-1, Math.min(1, power));
        verticalSlides.setPower(power);
        lastError = error;
        lastTime = System.nanoTime();
    }

    public void setSlidesPower(double power) {
        verticalSlides.setPower(power);
    }

    public void moveLift(double position) {
        position = Math.max(0.0, Math.min(1.0, position));
        liftServoLeft.setPosition(position);
        liftServoRight.setPosition(position);
    }

    public void moveWrist(double position) {
        wristServo.setPosition(Math.max(0.0, Math.min(1.0, position)));
    }

    public void setClawOpen(boolean open) {
        clawServo.setPosition(open ? 1.0 : 0.3);
    }

    private double inchesToTicks(double inches) {
        return inches * 28.229;
    }

    public void telemetry(Telemetry telemetry) {
        telemetry.addData("Vertical Slides Power", verticalSlides.getPower());
        telemetry.addData("Lift Pos L", liftServoLeft.getPosition());
        telemetry.addData("Lift Pos R", liftServoRight.getPosition());
        telemetry.addData("Wrist Pos", wristServo.getPosition());
        telemetry.addData("Claw Pos", clawServo.getPosition());
    }
}
