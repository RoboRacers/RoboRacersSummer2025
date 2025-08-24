package org.firstinspires.ftc.teamcode.layers.logicaldevice;

import org.firstinspires.ftc.teamcode.layers.physicaldevices.ContinuousMotorPhysicalBaseImpl;

public class ContinuousMotorLogicalBaseImpl implements ContinuousMotorLogicalIFC {

    private ContinuousMotorPhysicalBaseImpl continuousMotorPhysical = null;


    @Override
    public void initialize() {

    }

    @Override
    public void setPosition(float posInInches) {
        // get the rpm and then figure out how to convert rpm to inches
    }

    @Override
    public float getPosition() {
        return 0;
    }

    @Override
    public float getPowerUsed() {
        return 0;
    }

    @Override
    public void setPower(float power) {

    }

    @Override
    public void homeDevice() {

    }
}
