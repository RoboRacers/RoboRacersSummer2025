package org.firstinspires.ftc.teamcode.PostLobsterCup.example.layers.physicaldevices;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorImpl;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;


/**
 * This class is responsible for sending the commands to physical motor
 * This class extends from DcMotorImpl, so any additional decorative things or
 * operations can be added in addition when the user want to use the DcMotorImpl's API
 */
public class ContinuousMotorImpl extends DcMotorImpl
{
    public ContinuousMotorImpl(DcMotorController controller, int portNumber) {
        super(controller, portNumber, Direction.FORWARD);
    }

    public ContinuousMotorImpl(DcMotorController controller, int portNumber, DcMotorSimple.Direction direction) {
        super(controller, portNumber, direction, MotorConfigurationType.getUnspecifiedMotorType());
    }

    public ContinuousMotorImpl(DcMotorController controller, int portNumber, DcMotorSimple.Direction direction, @NonNull MotorConfigurationType motorType)
    {
        super(controller, portNumber, direction, motorType);
    }

    public static ContinuousMotorImpl create(DcMotor dcMotor)
    {
        return new ContinuousMotorImpl(dcMotor.getController(), dcMotor.getPortNumber(), dcMotor.getDirection(), dcMotor.getMotorType());
    }

    public synchronized void setPower(double power) {
        // Adding logging message
        super.setPower(power);
    }

}
