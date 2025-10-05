package org.firstinspires.ftc.teamcode.decode.example;


import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.ServoImpl;


/**
 * This class is responsible for sending the commands to physical motor
 * This class extends from DcMotorImpl, so any additional decorative things or
 * operations can be added in addition when the user want to use the DcMotorImpl's API
 */
public class SmartServo extends ServoImpl
{
    public SmartServo(ServoController controller, int portNumber, Servo.Direction direction,double max, double min,double restrictMax, double restrictMin) {
        super(controller, portNumber, direction);
        super.scaleRange(restrictMin,restrictMax);
    }


    public static SmartServo create(Servo servo, double max, double min)
    {
        double restrictMax = 1.0;
        double restrictMin= 0.0;
        return new SmartServo(servo.getController(), servo.getPortNumber(), servo.getDirection(), max, min, restrictMax, restrictMin);
    }

    public static SmartServo create(Servo servo, double max, double min, double restrictMax, double restrictMin )
    {

        return new SmartServo(servo.getController(), servo.getPortNumber(), servo.getDirection(), max, min, restrictMax, restrictMin);
    }

    public synchronized void setPosition(double angle) {
        // Adding logging message
        super.setPosition(angle);
        System.out.println("Setting power to: " + angle);

    }

}
