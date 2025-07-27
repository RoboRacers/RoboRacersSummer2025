package org.firstinspires.ftc.teamcode.teleop.LobsterCup;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class RobotControl {
    // Motors and servos
    public DcMotor intakeSlide, depositSlide;
    private Servo intakeClaw, depositClaw;
    private Servo wrist, lift, turret;
    private DigitalChannel limitSwitch;

    // Constants
    private static final double TICKS_PER_INCH = 28.229;

    public void init(HardwareMap hardwareMap) {
        intakeSlide = hardwareMap.get(DcMotor.class, "intakeSlide");
        depositSlide = hardwareMap.get(DcMotor.class, "depositSlide");

        intakeClaw = hardwareMap.get(Servo.class, "intakeClaw");
        depositClaw = hardwareMap.get(Servo.class, "depositClaw");
        wrist = hardwareMap.get(Servo.class, "wrist");
        lift = hardwareMap.get(Servo.class, "lift");
        turret = hardwareMap.get(Servo.class, "turret");

        limitSwitch = hardwareMap.get(DigitalChannel.class, "limitSwitch");

        intakeSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        depositSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        depositSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        depositSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        depositSlide.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public int inchesToTicks(double inches) {
        return (int)(inches * TICKS_PER_INCH);
    }

    // ========== Intake Functions ==========
    public void openIntakeClaw() {
        intakeClaw.setPosition(1.0);
    }

    public void closeIntakeClaw() {
        intakeClaw.setPosition(0.3);
    }

    public void setIntakeSlideInches(double inches) {
        int ticks = inchesToTicks(inches);
        intakeSlide.setTargetPosition(ticks);
        intakeSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intakeSlide.setPower(1);
    }

    // ========== Deposit Functions ==========
    public void openDepositClaw() {
        depositClaw.setPosition(1.0);
    }

    public void closeDepositClaw() {
        depositClaw.setPosition(0.3);
    }

    public void setDepositSlideInches(double inches) {
        int ticks = inchesToTicks(inches);
        depositSlide.setTargetPosition(ticks);
        depositSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        depositSlide.setPower(1);
    }

    public void setLift(double pos) {
        lift.setPosition(pos);
    }

    public void setWrist(double pos) {
        wrist.setPosition(pos);
    }

    public void setTurret(double pos) {
        turret.setPosition(pos);
    }

    // ========== Combined Actions ==========
    public void pickupSpecimen() {
        closeIntakeClaw();
        setWrist(0.5);
        setIntakeSlideInches(12);
        setLift(0.2);
    }

    public void dropSpecimen() {
        setDepositSlideInches(15);
        setLift(0.5);
        setWrist(0.8);
        openDepositClaw();
    }

    public void retractIntakeSlideIfLimitHit() {
        if (!limitSwitch.getState()) {
            intakeSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            intakeSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
}
