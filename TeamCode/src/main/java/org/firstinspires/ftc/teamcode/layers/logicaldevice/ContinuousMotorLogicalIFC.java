package org.firstinspires.ftc.teamcode.layers.logicaldevice;

public interface ContinuousMotorLogicalIFC{
    public void initialize();
    public void setPosition(float posInInches);
    public float getPosition();
    public float getPowerUsed();
    public void setPower(float power);
    public void homeDevice();
}
