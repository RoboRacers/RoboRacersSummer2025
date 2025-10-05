package org.firstinspires.ftc.teamcode.decode.example.layers.physicaldevices;


import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.ServoImpl;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.ServoConfigurationType;

import org.firstinspires.ftc.robotcore.external.Telemetry;


/**
 * This class is responsible for sending the commands to physical motor
 * This class extends from DcMotorImpl, so any additional decorative things or
 * operations can be added in addition when the user want to use the DcMotorImpl's API
 */
public class SmartServo extends ServoImpl
{

    public SmartServo(ServoController controller, int portNumber) {
        super(controller, portNumber);
    }
    public SmartServo(ServoController controller, int portNumber, Servo.Direction direction) {
        super(controller, portNumber, direction);
    }


    public static SmartServo create(Servo servo)
    {


        return new SmartServo(servo.getController(), servo.getPortNumber());
    }

    public synchronized void setAngle(double angle, double minAngle, double maxAngle) {
        // Adding logging message
       double position = Math.max(minAngle, Math.min(angle,maxAngle));
       setPos(position);
    }

    public synchronized void setAngle(double angle, double minAngle, double maxAngle, double restrictMin, double restrictMax) {
        // Adding logging message
        super.scaleRange(restrictMin,restrictMax);
        double position = Math.max(minAngle, Math.min(angle,maxAngle));
        setPos(position);
    }

    public synchronized void setPos(double pos) {
        // Adding logging message
        super.setPosition(pos);
    }

}
