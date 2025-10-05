package org.firstinspires.ftc.teamcode.decode.example.layers.coordinators;

import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.decode.example.layers.logicaldevices.CRServoLogical;
import org.firstinspires.ftc.teamcode.decode.example.layers.physicaldevices.SmartCRServo;

/** This class is responsible for coordinating the shooting.
 * The client has to simply call the API shoot(inches) to shoot
 */
public class CRServoCoord {

    private SmartCRServo gecoWheelServo;
    private CRServoLogical shooterLogical;

    public CRServoCoord(CRServo crServo, Telemetry tele)
    {
        gecoWheelServo = SmartCRServo.create(crServo, tele); // Get the servo from hardware map
        shooterLogical = new CRServoLogical(gecoWheelServo);
    }

    public void initialize()
    {
        // Make sure the servos used by this class are initialized
    }

    public void shoot(int distanceInches)
    {
        // TODO: Add logging of the distance to shoot given to logical layer

        // TODO:
        //  The GeckoWheel must be started
        //  The coordinator must wait for the wheels to be up to target speed.
        //  Then the servo to push the ball to the shooter
        //  After the ball is shot, the GeckoWheel should stop.

        shooterLogical.shoot(distanceInches);
    }
}
