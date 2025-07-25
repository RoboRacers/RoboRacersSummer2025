package org.firstinspires.ftc.teamcode.teleop.LobsterCupTeleop;

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

    public void init(HardwareMap hardwareMap) {
        verticalSlides = hardwareMap.get(DcMotor.class, "vSlides");
        liftServoLeft  = hardwareMap.get(Servo.class, "liftL");
        liftServoRight = hardwareMap.get(Servo.class, "liftR");
        wristServo     = hardwareMap.get(Servo.class, "wrist");
        clawServo      = hardwareMap.get(Servo.class, "cS"); // Shared with Intake

        verticalSlides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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
        position = Math.max(0.0, Math.min(1.0, position));
        wristServo.setPosition(position);
    }

    public void setClawOpen(boolean open) {
        clawServo.setPosition(open ? 1.0 : 0.3);
    }

    public void telemetry(Telemetry telemetry) {
        telemetry.addData("Vertical Slides Power", verticalSlides.getPower());
        telemetry.addData("Lift Pos L", liftServoLeft.getPosition());
        telemetry.addData("Lift Pos R", liftServoRight.getPosition());
        telemetry.addData("Wrist Pos", wristServo.getPosition());
        telemetry.addData("Claw Pos", clawServo.getPosition());
    }
}
