package org.firstinspires.ftc.teamcode.PostLobsterCup.logical;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImpl;

import org.firstinspires.ftc.robotcore.external.Telemetry;

// purpose: to control the turret by converting logical input (angle) into machine control input (hardware commands)
public class TurretLogical implements ServoLogicalIFC{

    private Servo turretPhysical;

    public TurretLogical (HardwareMap hardwareMap, Telemetry telemetry, String servoName) {
        turretPhysical = hardwareMap.get(Servo.class, servoName);
        myTelemetry = telemetry;
    }


    @Override
    public void setPosition(double angleDeg) throws Exception {

    }

    @Override
    public double getPosition() throws Exception {
        return 0;
    }

    @Override
    public String getName() {
        return "";
    }

    @Override
    public DoublePair getHardLimit() {
        return null;
    }

    @Override
    public void setHardLimit(double minAngle, double maxAngle) throws Exception {

    }

    @Override
    public DoublePair getSoftLimit() {
        return null;
    }

    @Override
    public void setSoftLimit(double minAngle, double maxAngle) throws Exception {

    }

    @Override
    public void homeDevice() throws Exception {

    }
}
