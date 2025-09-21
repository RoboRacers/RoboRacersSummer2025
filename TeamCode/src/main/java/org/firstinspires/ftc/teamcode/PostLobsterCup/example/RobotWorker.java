package org.firstinspires.ftc.teamcode.PostLobsterCup.example;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

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

    @Override
    public void runOpMode() throws InterruptedException
    {

        logMessage("Started the Robot Worker");

        shooter =  hardwareMap.get(DcMotor.class, "GecoWheelMotor");

        shooterCoord = new ShooterCoord(shooter);

        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive())
        {
            shooterCoord.shoot(6); // distance in Feet
            logMessage("Running Shooter");

        }
    }

    private void logMessage(String message)
    {
        String timeStamp = new SimpleDateFormat("yyyyMMdd_HHmmss").format(Calendar.getInstance().getTime());

        telemetry.addData(timeStamp, message);
        telemetry.update();
    }
}
