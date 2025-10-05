package org.firstinspires.ftc.teamcode.PostLobsterCup.example;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.configuration.MotorType;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.PostLobsterCup.example.layers.coordinators.ShooterCoord;

import java.text.SimpleDateFormat;
import java.util.Calendar;

/**
 * This class is the main class for the robot.
 * This class orchestrates all the subsystems from here.
 *
 */

@TeleOp(name="RobotWorker", group="Linear Opmode")
public class RobotWorker extends LinearOpMode {


    private DcMotor shooter; // Shooter

    private ShooterCoord shooterCoord;

    Telemetry mytele;

    private static int SHOOTER_MAX_RPM = 312;

    @Override
    public void runOpMode() throws InterruptedException
    {

        logMessage("Started the Robot Worker");

        shooter = getShooterMotor();

        mytele = telemetry;

        shooterCoord = new ShooterCoord(shooter, mytele);

        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive())
        {
            shooterCoord.shoot(6); // distance in Feet
            logMessage("Running Shooter");

        }
    }

    private DcMotor getShooterMotor() {
        DcMotor shooterMotor =  hardwareMap.get(DcMotor.class, "GecoWheelMotor");
        MotorConfigurationType motorType = shooterMotor.getMotorType();
        motorType.setMaxRPM(SHOOTER_MAX_RPM);
        shooterMotor.setMotorType(motorType);
        return shooterMotor;
    }

    private void logMessage(String message)
    {
        String timeStamp = new SimpleDateFormat("yyyyMMdd_HHmmss").format(Calendar.getInstance().getTime());

        telemetry.addData(timeStamp, message);
        telemetry.update();
    }
}
