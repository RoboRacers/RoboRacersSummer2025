package org.firstinspires.ftc.teamcode.layers.physicaldevices;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorImpl;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

/**
 * This class is a wrapper for physical device DCMotorImpl
 */
public class ContinuousMotorPhysicalBaseImpl extends DcMotorImpl {


    public ContinuousMotorPhysicalBaseImpl(DcMotorController controller, int portNumber) {
        this(controller, portNumber, Direction.FORWARD, MotorConfigurationType.getUnspecifiedMotorType());
    }

    public ContinuousMotorPhysicalBaseImpl(DcMotorController controller, int portNumber, Direction direction) {
        this(controller, portNumber, direction, MotorConfigurationType.getUnspecifiedMotorType());
    }

    public ContinuousMotorPhysicalBaseImpl(DcMotorController controller, int portNumber, Direction direction, @NonNull MotorConfigurationType motorType) {
        super(controller, portNumber, direction, motorType);
    }

}
