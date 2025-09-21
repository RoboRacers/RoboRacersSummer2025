package org.firstinspires.ftc.teamcode.PostLobsterCup.example.layers.coordinators;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.PostLobsterCup.example.layers.logicaldevices.ShooterLogical;
import org.firstinspires.ftc.teamcode.PostLobsterCup.example.layers.physicaldevices.ContinuousMotorImpl;

/** This class is responsible for coordinating the shooring.
 * The client have to simply call the API shoot(inches) to shoot
 */
public class ShooterCoord {

    private ContinuousMotorImpl gecoWheelMotor;
    private ShooterLogical shooterLogical;

    public ShooterCoord(DcMotor dcMotor)
    {
        gecoWheelMotor = ContinuousMotorImpl.create(dcMotor); // Get the motor from hardware map
        shooterLogical = new ShooterLogical(gecoWheelMotor);
    }

    public void initialize()
    {
        // Make sure the motors used by this class is initialized

    }

    public void shoot(int distanceInches)
    {
        // TODO: Add logging of the distance to shoot given to logical layer

        // TODO:
        //  The GeckoWheel must be started
        //  The coordinator must wait for the wheels to be up to target speed.
        //  Then the servo to push the ball to the shooter
        //  After the ball is shoot, the GeckoWheel should stop.

        shooterLogical.shoot(distanceInches);


    }
}
