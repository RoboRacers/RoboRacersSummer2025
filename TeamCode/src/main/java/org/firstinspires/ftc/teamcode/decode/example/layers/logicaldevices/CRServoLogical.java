package org.firstinspires.ftc.teamcode.decode.example.layers.logicaldevices;

import org.firstinspires.ftc.teamcode.decode.example.layers.physicaldevices.SmartCRServo;

/**
 * This class is responsible for computing the RPM/Power needed to shoot the
 * Ball for the given distance.
 */
public class CRServoLogical {

    private SmartCRServo gecoWheelServo;

    public CRServoLogical(SmartCRServo gecoWheelServoParam)
    {
        super();
        this.gecoWheelServo = gecoWheelServoParam;
    }

    public void initialize()
    {
        // If needed Initialize the servos used in this class;
    }

    public void shoot(int distanceInches)
    {
        double rpm = 60;
        // TODO: Math to compute RPM/Power needed to be set in servo to shoot the Ball to target distance

        gecoWheelServo.setRPM(rpm);

        // TODO: Log that the servo has been given the power to shoot
    }
}
