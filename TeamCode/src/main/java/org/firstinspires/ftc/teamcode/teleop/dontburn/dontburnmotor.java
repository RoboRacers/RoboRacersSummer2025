package org.firstinspires.ftc.teamcode.teleop.dontburn;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "dontburn DcMotor with Stall Protection", group = "Examples")
public class dontburnmotor extends LinearOpMode {

    private DcMotor motor;
    private static final String MOTOR_NAME = "myMotor"; // Change to your motor's configured name

    // Motor control variables
    private double targetMotorPower = 0.0;
    private double lastCommandedMotorPower = 0.0; // To detect if command has changed

    // Stall prevention variables
    private ElapsedTime stallTimer = new ElapsedTime();
    private static final double STALL_POWER_THRESHOLD = 0.2; // Minimum power to consider for stall detection
    // Don't check for stall if power is very low
    private static final double STALL_TIMEOUT_SECONDS = 1.5; // If applying significant power for this long
    // with no change in *commanded* power, suspect stall
    private boolean isPotentiallyStalled = false;

    // Optional: For more advanced stall detection using encoders
    private boolean useEncoderStallDetection = false; // Set to true if you have encoders and want to use them
    private static final int ENCODER_STALL_NO_CHANGE_THRESHOLD_TICKS = 5; // If encoder pos changes by less than this
    private int lastEncoderPosition = 0;


    @Override
    public void runOpMode() throws InterruptedException {
        motor = hardwareMap.get(DcMotor.class, MOTOR_NAME);

        // Set motor direction if needed (e.g., motor.setDirection(DcMotorSimple.Direction.REVERSE))
        motor.setDirection(DcMotorSimple.Direction.FORWARD);

        // Set ZeroPowerBehavior
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // Or FLOAT

        if (useEncoderStallDetection) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // Or RUN_USING_ENCODER if you primarily use encoders
            lastEncoderPosition = motor.getCurrentPosition();
            telemetry.addLine("Using ENCODER based stall detection.");
        } else {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // Typical for power control
            telemetry.addLine("Using POWER command based stall detection.");
        }

        stallTimer.reset();

        telemetry.addData("Motor", MOTOR_NAME);
        telemetry.addData("Stall Timeout (s)", STALL_TIMEOUT_SECONDS);
        telemetry.addLine("Gamepad 1 Left Stick Y controls motor power.");
        telemetry.addLine("Press 'A' on Gamepad 1 to reset stall if detected.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // --- Get Input for Target Motor Power ---
            // Example: Using gamepad left stick for power
            double rawPowerInput = -gamepad1.left_stick_y; // Negate for typical forward stick = forward motion

            // Small deadband for joystick
            if (Math.abs(rawPowerInput) < 0.05) {
                targetMotorPower = 0.0;
            } else {
                targetMotorPower = rawPowerInput;
            }

            // If user presses 'A' and motor is stalled, allow reset
            if (gamepad1.a && isPotentiallyStalled) {
                isPotentiallyStalled = false;
                stallTimer.reset(); // Reset timer
                // lastCommandedMotorPower will be updated below, allowing motor to run again
                telemetry.addLine("Stall manually reset by user.");
            }

            // --- Stall Detection and Motor Command Logic ---
            if (isPotentiallyStalled) {
                motor.setPower(0); // Ensure motor is stopped if stalled
                telemetry.addLine("*** MOTOR POTENTIALLY STALLED! ***");
                telemetry.addLine("Power set to 0. Press 'A' to reset and try again.");
            } else {
                // Not currently marked as stalled

                // Has the *commanded* power level changed significantly?
                // Or is the current command significant enough to potentially cause a stall?
                boolean significantPowerChange = Math.abs(targetMotorPower - lastCommandedMotorPower) > 0.01;
                boolean applyingSignificantPower = Math.abs(targetMotorPower) >= STALL_POWER_THRESHOLD;

                if (significantPowerChange) {
                    // Commanded power has changed, apply it and reset stall timer
                    motor.setPower(targetMotorPower);
                    lastCommandedMotorPower = targetMotorPower;
                    stallTimer.reset();
                    if (useEncoderStallDetection) {
                        lastEncoderPosition = motor.getCurrentPosition();
                    }
                    // telemetry.addData("Motor Power", "%.2f (New Command)", targetMotorPower);
                } else if (applyingSignificantPower) {
                    // Commanded power has NOT changed, but we are applying significant power.
                    // This is where we check for a stall.

                    boolean stalledByEncoder = false;
                    if (useEncoderStallDetection) {
                        int currentEncoderPos = motor.getCurrentPosition();
                        if (Math.abs(currentEncoderPos - lastEncoderPosition) < ENCODER_STALL_NO_CHANGE_THRESHOLD_TICKS) {
                            // Encoder position hasn't changed much
                            stalledByEncoder = true;
                        } else {
                            lastEncoderPosition = currentEncoderPos; // Update if moving
                            stallTimer.reset(); // Reset stall timer if encoders show movement
                        }
                    }

                    // Check stall timer:
                    // - If using encoders, AND encoder shows no movement.
                    // - OR if not using encoders, just rely on time.
                    if (stallTimer.seconds() > STALL_TIMEOUT_SECONDS && (!useEncoderStallDetection || stalledByEncoder) ) {
                        telemetry.addLine(String.format("Stall suspected for '%s' (Power: %.2f) after %.1fs",
                                MOTOR_NAME, lastCommandedMotorPower, STALL_TIMEOUT_SECONDS));
                        if(useEncoderStallDetection && stalledByEncoder) telemetry.addLine("Encoder position also not changing.");
                        isPotentiallyStalled = true;
                        motor.setPower(0); // Stop motor immediately on stall detection
                    } else {
                        // Still applying same power, timer not expired OR encoders show movement
                        motor.setPower(targetMotorPower); // Continue applying current power
                        // telemetry.addData("Motor Power", "%.2f (Holding, Timer: %.1fs)", targetMotorPower, stallTimer.seconds());
                    }
                } else {
                    // Commanded power is low or zero, and hasn't changed.
                    // Not considered a stall risk.
                    motor.setPower(targetMotorPower);
                    lastCommandedMotorPower = targetMotorPower; // Keep lastCommanded up to date
                    stallTimer.reset(); // Reset timer if power is low/zero
                    if (useEncoderStallDetection) {
                        lastEncoderPosition = motor.getCurrentPosition(); // Keep encoder pos updated
                    }
                }
            }

            // --- Telemetry ---
            telemetry.addData("Target Power", "%.2f", targetMotorPower);
            telemetry.addData("Last Cmd Power", "%.2f", lastCommandedMotorPower);
            telemetry.addData("Is Stalled?", isPotentiallyStalled);
            telemetry.addData("Stall Timer (s)", "%.2f / %.1f", stallTimer.seconds(), STALL_TIMEOUT_SECONDS);
            if (useEncoderStallDetection) {
                telemetry.addData("Encoder Pos", motor.getCurrentPosition());
                telemetry.addData("Last Enc Pos", lastEncoderPosition);
            }
            telemetry.update();

            sleep(20); // Loop delay
        }
        motor.setPower(0); // Ensure motor is off when OpMode stops
    }
}
