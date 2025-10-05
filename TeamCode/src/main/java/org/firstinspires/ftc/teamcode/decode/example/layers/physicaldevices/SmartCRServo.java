package org.firstinspires.ftc.teamcode.decode.example.layers.physicaldevices;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImpl;
import com.qualcomm.robotcore.hardware.ServoController;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * This class is responsible for sending the commands to physical continuous rotation servo.
 * This class extends from CRServoImpl, so any additional decorative things or
 * operations can be added in addition when the user want to use the CRServoImpl's API.
 */
public class SmartCRServo extends CRServoImpl
{
    public SmartCRServo(ServoController controller, int portNumber) {
        super(controller, portNumber, Direction.FORWARD);
    }

    public SmartCRServo(ServoController controller, int portNumber, Direction direction) {
        super(controller, portNumber, direction);
    }

    public SmartCRServo(ServoController controller, int portNumber, Direction direction, @NonNull Object ignoredMotorType)
    {
        super(controller, portNumber, direction);
    }

    public static SmartCRServo create(CRServo crServo, Telemetry telemetry)
    {
        return new SmartCRServo(crServo.getController(), crServo.getPortNumber(), crServo.getDirection(), null);
    }

    public synchronized void setRPM(double rpm){
        try{
            // CRServos do not have max RPM info, so let's just map rpm to power
            // Assume max RPM = 100 for example; you can adjust this value as needed
            double maxRPM = 100.0;
            if (rpm > maxRPM){
                throw new Exception("RPM is greater than max RPM. Max RPM: " + maxRPM);
            }else if (rpm < maxRPM){
                setPower(rpm/maxRPM);
            }

        }catch (Exception e){
            telemetry.addData("Error", e.getMessage());
            setPower(1);
        }
    }

    @Override
    public synchronized void setPower(double power) {
        // Adding logging message
        super.setPower(power);
    }
}
