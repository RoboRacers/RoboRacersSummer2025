package org.firstinspires.ftc.teamcode.PostLobsterCup.logical;

public interface ServoLogicalIFC
{
    public void setPosition(double angleDeg) throws Exception; // Set the position in terms of angle in degrees
    public double getPosition() throws Exception; // Returns the position as angle in degrees
    public String getName();
    public DoublePair getHardLimit();
    public void setHardLimit(double minAngle, double maxAngle) throws Exception;
    public DoublePair getSoftLimit();
    public void setSoftLimit(double minAngle, double maxAngle) throws Exception
    public void homeDevice() throws Exception;

}
