package org.firstinspires.ftc.teamcode.teleop.LobsterCup;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake {
    private Servo heightServo;
    private Servo rotateServo;
    public Servo clawServo;
    private Servo turret;
    private DcMotor slidesMotor;

    private double heightPos = 0.5;
    private double rotatePos = 0.5;
    private double wristPos = 0.5;

    private double targetInches = 0;
    private double kP = 0.02, kI = 0.0000000001, kD = 0.0000000001;
    private double integralSum = 0, lastError = 0;
    private long lastTime = System.nanoTime();

    public void init(HardwareMap hardwareMap) {
        heightServo = hardwareMap.get(Servo.class, "heightServo");
        rotateServo = hardwareMap.get(Servo.class, "rotateServo");
        clawServo   = hardwareMap.get(Servo.class, "clawServo");
        turret = hardwareMap.get(Servo.class, "turret");
        slidesMotor = hardwareMap.get(DcMotor.class, "intakeSlide");

        heightServo.setPosition(heightPos);
        rotateServo.setPosition(rotatePos);
        clawServo.setPosition(0.3); // Closed

        slidesMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slidesMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidesMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setSlidesTargetInches(double inches) {
        targetInches = inches;
    }

    public void update() {
        double ticks = targetInches;
        double current = slidesMotor.getCurrentPosition();
        double error = ticks - current;
        double dt = (System.nanoTime() - lastTime) / 1e9;
        integralSum += error * dt;
        double derivative = (error - lastError) / dt;
        double power = kP * error + kI * integralSum + kD * derivative;
        power = Math.max(-1, Math.min(1, power));
        slidesMotor.setPower(power);
        lastError = error;
        lastTime = System.nanoTime();
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

    private double inchesToTicks(double inches) {
        return inches * 28.229;
    }

    public void telemetry(Telemetry telemetry) {
        telemetry.addData("Horizontal Slides Power", slidesMotor.getPower());
        telemetry.addData("Horizontal Slides pos", slidesMotor.getCurrentPosition());

        telemetry.addData("Turret Pos", turret.getPosition());
        telemetry.addData("Arm Pos", heightServo.getPosition());
        telemetry.addData("Rotate Servo Pos", rotateServo.getPosition());
        telemetry.addData("Claw Pos", clawServo.getPosition());
    }
}
