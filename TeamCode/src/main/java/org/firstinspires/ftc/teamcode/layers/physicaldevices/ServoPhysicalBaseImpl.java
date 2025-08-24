package org.firstinspires.ftc.teamcode.layers.physicaldevices;

import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.ServoImpl;

import org.firstinspires.ftc.teamcode.PostLobsterCup.utilities.DoublePair;
import org.firstinspires.ftc.teamcode.PostLobsterCup.utilities.FloatPair;

/**
 * This class is responsible for communicating with the Physical Motor hardware.
 * Since the SDK already provides API, we simply extend from it to add more
 * API on top of that.
 */
public class ServoPhysicalBaseImpl extends ServoImpl
{
    private FloatPair motorCntRange;
    private FloatPair rangeInDeg;

    public ServoPhysicalBaseImpl(ServoController controller, int portNumber, FloatPair motorCntRangeParam, FloatPair rangeInDegParam) {
        this(controller, portNumber, Direction.FORWARD, motorCntRangeParam, rangeInDegParam);
    }

    public ServoPhysicalBaseImpl(ServoController controller, int portNumber, Direction direction, FloatPair motorCntRangeParam, FloatPair rangeInDegParam) {
        super(controller, portNumber, direction);

        this.motorCntRange = motorCntRangeParam;
        this.rangeInDeg = rangeInDegParam;
    }

    public DoublePair getHWLimits()
    {
        return new DoublePair(limitPositionMin, limitPositionMax);
    }

    public FloatPair getRangeInDeg() {
        return rangeInDeg;
    }

    public FloatPair getMotorCntRange() {
        return motorCntRange;
    }
}
