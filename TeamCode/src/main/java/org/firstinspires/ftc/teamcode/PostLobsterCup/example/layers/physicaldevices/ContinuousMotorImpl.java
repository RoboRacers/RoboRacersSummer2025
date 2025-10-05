package org.firstinspires.ftc.teamcode.PostLobsterCup.example.layers.physicaldevices;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorImpl;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.Telemetry;


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

    public static ContinuousMotorImpl create(DcMotor dcMotor, Telemetry telemetry)
    {
        //telemetry.addData("Motor Type", dcMotor.getMotorType());

        return new ContinuousMotorImpl(dcMotor.getController(), dcMotor.getPortNumber(), dcMotor.getDirection(), dcMotor.getMotorType());
    }

    public synchronized void setRPM(double rpm){
        try{
            if (rpm > this.motorType.getMaxRPM()){
                throw new Exception("RPM is greater than max RPM. Max RPM: " + this.motorType.getMaxRPM());
            }else if (rpm < this.motorType.getMaxRPM()){
                setPower(rpm/this.motorType.getMaxRPM());
            }

        }catch (Exception e){
            telemetry.addData("Error", e.getMessage());
            setPower(1);
        }
    }



    public synchronized void setPower(double power) {
        // Adding logging message
        super.setPower(power);
    }

}
