package org.firstinspires.ftc.teamcode.PostLobsterCup.physical.Drive;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DriveMotors {
    private DcMotorEx frontRight;
    private DcMotorEx frontLeft;
    private DcMotorEx backRight;
    private DcMotorEx backLeft;

    private Telemetry telemetry;

    public DriveMotors(HardwareMap hardwareMap, Telemetry telemetry, String frName, String flName, String brName, String blName) {
        frontRight = hardwareMap.get(DcMotorEx.class, frName);
        frontLeft = hardwareMap.get(DcMotorEx.class, flName);
        backRight = hardwareMap.get(DcMotorEx.class, brName);
        backLeft = hardwareMap.get(DcMotorEx.class, blName);

        this.telemetry = telemetry;

        // Reverse right side so forward is forward
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /** Sets the same power to all drive motors */
    public void setAllPower(double power) {
        frontRight.setPower(power);
        frontLeft.setPower(power);
        backRight.setPower(power);
        backLeft.setPower(power);
    }

    /** Stops all drive motors */
    public void stop() {
        setAllPower(0);
    }

    /** Mecanum drive control (forward, strafe, turn) */
    public void mecanumDrive(double forward, double strafe, double turn) {
        double fl = forward + strafe + turn;
        double fr = forward - strafe - turn;
        double bl = forward - strafe + turn;
        double br = forward + strafe - turn;
        frontLeft.setPower(fl);
        frontRight.setPower(fr);
        backLeft.setPower(bl);
        backRight.setPower(br);
    }

    /** Reset all motor encoders */
    public void resetEncoders() {
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    /** Set all motors to run using encoders */
    public void runUsingEncoders() {
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /** Set all motors to brake or coast when power is zero */
    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        frontRight.setZeroPowerBehavior(behavior);
        frontLeft.setZeroPowerBehavior(behavior);
        backRight.setZeroPowerBehavior(behavior);
        backLeft.setZeroPowerBehavior(behavior);
    }

    /** Send encoder positions to telemetry */
    public void reportEncoders() {
        telemetry.addData("frontRight Encoder", frontRight.getCurrentPosition());
        telemetry.addData("frontLeft Encoder", frontLeft.getCurrentPosition());
        telemetry.addData("backRight Encoder", backRight.getCurrentPosition());
        telemetry.addData("backLeft Encoder", backLeft.getCurrentPosition());
        telemetry.update();
    }
}
