package org.firstinspires.ftc.teamcode.teleop.LobsterCupTeleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.Arrays;
import java.util.List;

// Import Intake and Deposit subsystem classes
import org.firstinspires.ftc.teamcode.teleop.LobsterCupTeleop.Intake;
import org.firstinspires.ftc.teamcode.teleop.LobsterCupTeleop.Deposit;

/**
 * Gamepad Mapping Plan
 *
 * Gamepad 1 – Drivetrain (Mecanum Drive)
 * - left_stick_y: Forward/Backward (inverted)
 * - left_stick_x: Strafe Left/Right
 * - right_stick_x: Rotate
 * (Same as the driving in your example LocalizationTest)
 *
 * Gamepad 2 – Manipulators (Intake + Deposit)
 *
 * Intake Functions
 * Control               | Function
 * ---------------------|------------------------------
 * dpad_up / dpad_down  | Increase/decrease height servo
 * dpad_right / dpad_left | Increase/decrease rotate servo
 * x                    | Open claw
 * b                    | Close claw
 * left_stick_y         | Intake slides motor power (up/down)
 *
 * Deposit Functions
 * Control              | Function
 * --------------------|------------------------------
 * a                    | Open claw
 * y                    | Close claw
 * left_bumper          | Raise lift servos (setLift(1.0))
 * right_bumper         | Lower lift servos (setLift(0.0))
 * right_trigger        | Move wrist up (wrist += 0.01)
 * left_trigger         | Move wrist down (wrist -= 0.01)
 * right_stick_y        | Deposit slides motor power (up/down)
 */

@TeleOp(name = "Everything TeleOp", group = "Main")
public class FinalTeleo extends OpMode {

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
        // === DRIVETRAIN (gamepad 1) ===
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double lfPower = (y + x + rx) / denominator;
        double lrPower = (y - x + rx) / denominator;
        double rfPower = (y - x - rx) / denominator;
        double rrPower = (y + x - rx) / denominator;

        leftFront.setPower(lfPower);
        leftRear.setPower(lrPower);
        rightFront.setPower(rfPower);
        rightRear.setPower(rrPower);

        // === INTAKE (gamepad 2) ===

        // Height
        if (gamepad2.dpad_up) {
            heightPos += 0.01;
        } else if (gamepad2.dpad_down) {
            heightPos -= 0.01;
        }
        intake.setHeightPosition(clamp(heightPos));

        // Rotation
        if (gamepad2.dpad_right) {
            rotatePos += 0.01;
        } else if (gamepad2.dpad_left) {
            rotatePos -= 0.01;
        }
        intake.setRotatePosition(clamp(rotatePos));

        // Claw
        if (gamepad2.x) {
            intake.setClawOpen(true);
        } else if (gamepad2.b) {
            intake.setClawOpen(false);
        }

        // Slides
        intake.setSlidesPower(-gamepad2.left_stick_y);

        // === DEPOSIT (gamepad 2) ===

        // Deposit claw
        if (gamepad2.a) {
            deposit.setClawOpen(true);
        } else if (gamepad2.y) {
            deposit.setClawOpen(false);
        }

        // Lift servos
        if (gamepad2.left_bumper) {
            deposit.moveLift(1.0); // up
        } else if (gamepad2.right_bumper) {
            deposit.moveLift(0.0); // down
        }

        // Wrist
        if (gamepad2.right_trigger > 0.05) {
            wristPos += 0.01;
        } else if (gamepad2.left_trigger > 0.05) {
            wristPos -= 0.01;
        }
        deposit.moveWrist(clamp(wristPos));

        // Deposit slides
        deposit.setSlidesPower(-gamepad2.right_stick_y);
    }

    private double clamp(double value) {
        return Math.max(0.0, Math.min(1.0, value));
    }
}
