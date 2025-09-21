package org.firstinspires.ftc.teamcode.PostLobsterCup.example.layers.logicaldevices;

import org.firstinspires.ftc.teamcode.PostLobsterCup.example.layers.physicaldevices.ContinuousMotorImpl;

/**
 * This class is responsible for computing the RPM/Power needed to shoot the
 * Ball for the given distance.
 */
public class ShooterLogical {

    private ContinuousMotorImpl gecoWheelMotor;

    public ShooterLogical(ContinuousMotorImpl gecoWheelMotorParam)
    {
        super();
        this.gecoWheelMotor = gecoWheelMotorParam;
    }

    public void initialize()
    {
        // If needed Initialize the motors used in this class;

    }

    public void shoot(int distanceInches)
    {
        double power = 0.2d;
        // TODO: Math to compute RPM/Power needed to be set in motor to shoot the Ball to target distance

        gecoWheelMotor.setPower(power);


        // TODO: Log that the motor has been given the power to shoot
    }
}
