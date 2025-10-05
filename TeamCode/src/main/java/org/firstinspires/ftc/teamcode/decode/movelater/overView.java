package org.firstinspires.ftc.teamcode.decode.movelater;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.teleop.LobsterCup.Deposit;
import org.firstinspires.ftc.teamcode.teleop.LobsterCup.Intake;

import java.util.Arrays;
import java.util.List;

@TeleOp(name="everything", group="Linear Opmode")
public class overView extends OpMode {

    // Declare motors
    // Drivetrain
    private DcMotorEx leftFront, leftRear, rightFront, rightRear;
    private List<DcMotorEx> driveMotors;



    // Subsystems
    private Intake intake = new Intake();
    private Deposit deposit = new Deposit();

    // State vars for smooth increments
    private double heightPos = 0.5;
    private double rotatePos = 0.5;
    private double wristPos = 0.5;

    @Override
    public void init() {
        // Init drivetrain
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");

        driveMotors = Arrays.asList(leftFront, leftRear, rightFront, rightRear);
        for (DcMotorEx motor : driveMotors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        // Init subsystems
        intake.init(hardwareMap);
        deposit.init(hardwareMap);
    }

    @Override
    public void loop() {

    }
}
