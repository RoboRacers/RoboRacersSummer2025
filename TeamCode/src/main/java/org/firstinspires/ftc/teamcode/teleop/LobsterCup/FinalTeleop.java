package org.firstinspires.ftc.teamcode.teleop.LobsterCup;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;


import java.util.Arrays;
import java.util.List;

/**
 * Gamepad Mapping Plan
 *
 * Gamepad 1 – Drivetrain (Mecanum Drive)
 * - left_stick_y: Forward/Backward (inverted)
 * - left_stick_x: Strafe Left/Right
 * - right_stick_x: Rotate
 *
 * Gamepad 2 – Manipulators (Intake + Deposit)
 *
 * Intake
 * dpad_up/down     → Height Servo ↑↓
 * dpad_left/right  → Rotate Servo(turret) ←→
 * left_stick_x
 * x/b              → Claw open/close
 * left_stick_y     → Intake Slide Motor
 *
 * Deposit
 * a/y              → Deposit Claw open/close
 * left/right_bumper→ Lift Servos ↑↓
 * left/right_trigger → Wrist Servo ↓↑
 * right_stick_y    → Deposit Slide Motor
 *
 * Hardware Configuration (Driver Hub > Robot Config):
 *
 * Motors:
 * - "leftFront"     → DcMotorEx
 * - "leftRear"      → DcMotorEx
 * - "rightFront"    → DcMotorEx
 * - "rightRear"     → DcMotorEx
 * - "intakeSlide"   → DcMotorEx
 * - "depositSlide"  → DcMotorEx
 *
 * Servos:
 * - "heightServo"   → Servo
 * - "rotateServo"   → Servo
 * - "clawServo"     → Servo (on Intake)
 * - "depositClaw"   → Servo
 * - "liftLeft"      → Servo
 * - "liftRight"     → Servo
 * - "wristServo"    → Servo(on Deposit)
 */

@TeleOp(name = "LobsterCup TeleOp (For my goat Ishaan ~ Mith)", group = "Main")
public class FinalTeleop extends OpMode {

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
        if (gamepad2.dpad_up) heightPos += 0.01;
        else if (gamepad2.dpad_down) heightPos -= 0.01;
        intake.setHeightPosition(clamp(heightPos));

        // Rotation
        if (gamepad2.dpad_right) rotatePos += 0.01;
        else if (gamepad2.dpad_left) rotatePos -= 0.01;
        intake.setRotatePosition(clamp(rotatePos));

        // Claw
        if (gamepad2.x) intake.setClawOpen(true);
        else if (gamepad2.b) intake.setClawOpen(false);

        // Intake Wrist
        intake.setWristIntake(-gamepad2.left_stick_x);

        // Slides – PID Target
        intake.setSlidesTargetInches(-gamepad2.left_stick_y * 10);  // map joystick to 0–10in range

        // === DEPOSIT (gamepad 2) ===

        // Claw
        if (gamepad2.a) deposit.setClawOpen(true);
        else if (gamepad2.y) deposit.setClawOpen(false);

        // Lift
        if (gamepad2.left_bumper) deposit.moveLift(1.0);
        else if (gamepad2.right_bumper) deposit.moveLift(0.0);

        // Wrist
        if (gamepad2.right_trigger > 0.05) wristPos += 0.01;
        else if (gamepad2.left_trigger > 0.05) wristPos -= 0.01;
        deposit.moveWrist(clamp(wristPos));

        // Slides – PID Target
        deposit.setSlidesTargetInches(-gamepad2.right_stick_y * 10);

        // === PID UPDATES ===
        intake.update();
        deposit.update();

        // === TELEMETRY ===
        telemetry.addLine("🎮 Gamepad 2 Controls");
        telemetry.addData("Height Servo", heightPos);
        telemetry.addData("Rotate Servo", rotatePos);
        telemetry.addData("Wrist Servo", wristPos);
        telemetry.addData("Intake Slide Target (in)", -gamepad2.left_stick_y * 10);
        telemetry.addData("Deposit Slide Target (in)", -gamepad2.right_stick_y * 10);

        telemetry.addData("Intake Claw", gamepad2.x ? "Open" : (gamepad2.b ? "Close" : "Idle"));
        telemetry.addData("Deposit Claw", gamepad2.a ? "Open" : (gamepad2.y ? "Close" : "Idle"));
        telemetry.addData("Lift", gamepad2.left_bumper ? "Up" : (gamepad2.right_bumper ? "Down" : "Idle"));
        telemetry.addData("Wrist", gamepad2.right_trigger > 0.05 ? "Up" : (gamepad2.left_trigger > 0.05 ? "Down" : "Idle"));

        telemetry.update();
    }

    private double clamp(double value) {
        return Math.max(0.0, Math.min(1.0, value));
    }
}
