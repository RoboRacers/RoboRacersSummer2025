package org.firstinspires.ftc.teamcode.decode.example;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class configBank {

    private DcMotor shooterMotor; // Shooter

    private DcMotorEx frontRight;
    private DcMotorEx frontLeft;
    private DcMotorEx backRight;
    private DcMotorEx backLeft;

    private static int SHOOTER_MAX_RPM = 312;

    private Telemetry telemetry;
    public configBank(HardwareMap hardwareMap, Telemetry telemetry){
        frontRight = hardwareMap.get(DcMotorEx.class, "frName");
        frontLeft = hardwareMap.get(DcMotorEx.class, "flName");
        backRight = hardwareMap.get(DcMotorEx.class, "brName");
        backLeft = hardwareMap.get(DcMotorEx.class, "blName");



        this.telemetry = telemetry;

        // Reverse right side so forward is forward
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        shooterMotor =  hardwareMap.get(DcMotor.class, "GecoWheelMotor");
        MotorConfigurationType motorType = shooterMotor.getMotorType();
        motorType.setMaxRPM(SHOOTER_MAX_RPM);
        shooterMotor.setMotorType(motorType);
    }
    public DcMotor getShooter(){
        return this.shooterMotor;
    }



}
