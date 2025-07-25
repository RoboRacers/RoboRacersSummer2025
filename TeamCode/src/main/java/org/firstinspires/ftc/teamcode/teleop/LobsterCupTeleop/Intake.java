package org.firstinspires.ftc.teamcode.teleop.LobsterCupTeleop;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    private Servo heightServo;
    private Servo rotateServo;
    private Servo clawServo;
    private DcMotor slidesMotor;

    private double heightPos = 0.5;
    private double rotatePos = 0.5;

    public void init(HardwareMap hardwareMap) {
        heightServo = hardwareMap.get(Servo.class, "hS");
        rotateServo = hardwareMap.get(Servo.class, "rS");
        clawServo   = hardwareMap.get(Servo.class, "cS");
        slidesMotor = hardwareMap.get(DcMotor.class, "slides");

        heightServo.setPosition(heightPos);
        rotateServo.setPosition(rotatePos);
        clawServo.setPosition(0.3); // Closed

        slidesMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    // Adjust height servo position
    public void setHeightPosition(double position) {
        heightPos = clamp(position);
        heightServo.setPosition(heightPos);
    }

    // Adjust rotation servo position
    public void setRotatePosition(double position) {
        rotatePos = clamp(position);
        rotateServo.setPosition(rotatePos);
    }

    // Open or close claw
    public void setClawOpen(boolean open) {
        clawServo.setPosition(open ? 1.0 : 0.3);
    }

    // Set slides motor power
    public void setSlidesPower(double power) {
        slidesMotor.setPower(power);
    }

    // Getters (optional for telemetry/debugging in master file)
    public double getHeightPosition() {
        return heightServo.getPosition();
    }

    public double getRotatePosition() {
        return rotateServo.getPosition();
    }

    public double getClawPosition() {
        return clawServo.getPosition();
    }

    public double getSlidesPower() {
        return slidesMotor.getPower();
    }

    private double clamp(double value) {
        return Math.max(0.0, Math.min(1.0, value));
    }
}
