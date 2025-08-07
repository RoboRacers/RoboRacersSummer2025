package org.firstinspires.ftc.teamcode.PostLobsterCup.Layer1.Drive;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * WORK IN PROGRESS :)
 */

public class DriveMotors {
    private DcMotorEx frontRight;
    private DcMotorEx frontLeft;
    private DcMotorEx backRight;
    private DcMotorEx backLeft;

    private Telemetry telemetry;

    private final int STOP = 0;
    private final int MOVE_FORWARD = 0;
    private final int MOVE_BACK = 0;

    public DriveMotors(HardwareMap hardwareMap, Telemetry telemetry, String motorName, String limitSwitchName) {
        frontRight = hardwareMap.get(DcMotorEx.class, motorName);
        frontLeft = hardwareMap.get(DcMotorEx.class, motorName);
        backRight = hardwareMap.get(DcMotorEx.class, motorName);
        backLeft = hardwareMap.get(DcMotorEx.class, motorName);

        this.telemetry = telemetry;

        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        
    }
}
