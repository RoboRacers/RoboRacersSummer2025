package org.firstinspires.ftc.teamcode.layers.logicaldevice;

import org.firstinspires.ftc.teamcode.PostLobsterCup.utilities.DoublePair;

/**
 * The role of this interface is to force the logical component to implement these APIs,
 * so that users can access the logical device using these standard APIs
 *
 */
public interface ServoLogicalIFC
{
    public void setPositionInDeg(double angleDeg) throws Exception; // Set the position in terms of angle in degrees
    public double getPositionInDeg() throws Exception; // Returns the position as angle in degrees
    public String getName();
    public DoublePair getSoftLimit();
    public void setSoftLimit(double minAngle, double maxAngle) throws Exception;
    public void homeDevice() throws Exception;
}
