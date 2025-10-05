package org.firstinspires.ftc.teamcode.layers.logicaldevice;

import org.firstinspires.ftc.teamcode.layers.physicaldevices.ServoPhysicalBaseImpl;
import org.firstinspires.ftc.teamcode.PostLobsterCup.utilities.DoublePair;
import org.firstinspires.ftc.teamcode.PostLobsterCup.utilities.FloatPair;

public class ServoLogicalBaseImpl implements ServoLogicalIFC{

    private ServoPhysicalBaseImpl servoMotorPhysical = null;
    private DoublePair softLimitInDeg;

    private String motorName = "";

    public ServoLogicalBaseImpl(ServoPhysicalBaseImpl servoPhysical, DoublePair softLimitInDegParam, String motorNameParam)
    {
        this.servoMotorPhysical = servoPhysical;
        this.softLimitInDeg = softLimitInDegParam;
        this.motorName = motorNameParam;
    }

    @Override
    /**
     * This method translates the given degrees to motor cnt and sets the motor to that position
     *
     */
    public void setPositionInDeg(double angleDeg) throws Exception {
        // NewValue = (((OldValue - OldMin) * (NewMax - NewMin)) / (OldMax - OldMin)) + NewMin

        // [RangeStartDeg, RangeEndDeg] - Servo Range in Degrees
        // [MotorCntRangeStart, MotorCntRangeEnd] - Servo Range in Motor Cnts (Typically 0 to 1)
        FloatPair rangeInDeg = servoMotorPhysical.getRangeInDeg();
        FloatPair rangeInMC = servoMotorPhysical.getMotorCntRange();

        float rangeStartDeg = rangeInDeg.getMin();
        float rangeEndDeg = rangeInDeg.getMax();
        float rangeStartMC = rangeInMC.getMin();
        float rangeEndMC = rangeInMC.getMax();

        double motorPos = (double) ( ( ((angleDeg - rangeStartDeg) * (rangeEndMC - rangeStartMC)) / (rangeEndDeg - rangeStartDeg)) + rangeStartMC);

        servoMotorPhysical.setPosition(motorPos);

    }

    /**
     * This method converts the motor position from motor counts to degrees
     * and returns motor position in degrees.
     */
    @Override
    public double getPositionInDeg() throws Exception {

        // [RangeStartDeg, RangeEndDeg] - Servo Range in Degrees
        // [MotorCntRangeStart, MotorCntRangeEnd] - Servo Range in Motor Cnts (Typically 0 to 1)
        FloatPair rangeInDeg = servoMotorPhysical.getRangeInDeg();
        FloatPair rangeInMC = servoMotorPhysical.getMotorCntRange();

        float rangeStartDeg = rangeInDeg.getMin();
        float rangeEndDeg = rangeInDeg.getMax();
        float rangeStartMC = rangeInMC.getMin();
        float rangeEndMC = rangeInMC.getMax();

        double currentPosInMC = servoMotorPhysical.getPosition();

        double angleInDeg = (double) (((currentPosInMC - rangeStartMC) * (rangeEndDeg - rangeStartDeg)) /  (rangeEndMC - rangeStartMC)) + rangeStartDeg;

        return angleInDeg;
    }

    @Override
    public String getName() {
        return motorName;
    }

    @Override
    public DoublePair getSoftLimit() {
        return softLimitInDeg;
    }

    @Override
    public void setSoftLimit(double minAngleInDeg, double maxAngleInDeg) throws Exception {
        softLimitInDeg = new DoublePair(minAngleInDeg, maxAngleInDeg);
    }

    @Override
    public void homeDevice() throws Exception {
        FloatPair motorCntRange = servoMotorPhysical.getMotorCntRange();
        servoMotorPhysical.setPosition(motorCntRange.getMin());
    }
}
