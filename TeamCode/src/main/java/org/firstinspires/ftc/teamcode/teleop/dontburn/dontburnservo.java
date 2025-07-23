package org.firstinspires.ftc.teamcode.teleop.dontburn;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime; // Import ElapsedTime

@TeleOp(name = "dontburn Servo with Protection") // Renamed for clarity
public class dontburnservo extends LinearOpMode {

    private Servo servo;
    private double currentServoPosition = 0.5;
    private double targetServoPosition = 0.5;

    private final double INCREMENT = 0.01;
    private final double MIN_POS = 0.0;
    private final double MAX_POS = 1.0;

    private ElapsedTime stallTimer = new ElapsedTime();
    private static final double STALL_TIMEOUT_SECONDS = 2.0;
    private static final double STALL_POSITION_THRESHOLD = 0.005;
    private boolean isPotentiallyStalled = false;
    private double lastNonStalledTargetPosition = 0.5;

    @Override
    public void runOpMode() {
        servo = hardwareMap.get(Servo.class, "servo");

        // Initialize servo to starting position
        servo.setPosition(currentServoPosition);
        lastNonStalledTargetPosition = currentServoPosition;
        double servopos = servo.getPosition();
        stallTimer.reset();

        telemetry.addLine("Ready - Press Play. Servo with Stall Protection.");
        telemetry.addData("Stall Timeout (s)", STALL_TIMEOUT_SECONDS);
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            double previousTargetServoPosition = targetServoPosition;

            if (gamepad1.right_trigger > 0.1) {
                targetServoPosition += INCREMENT;
                isPotentiallyStalled = false;
            }

            if (gamepad1.left_trigger > 0.1) {
                targetServoPosition -= INCREMENT;
                isPotentiallyStalled = false;
            }

            // Clamp the target position
            targetServoPosition = Math.max(MIN_POS, Math.min(MAX_POS, targetServoPosition));

            // --- Stall Detection Logic ---
            if (isPotentiallyStalled) {
                telemetry.addLine(" SERVO POTENTIALLY STALLED! ");
                telemetry.addLine(String.format("Was trying to reach %.2f. Trigger to set new target.", lastNonStalledTargetPosition));
            } else {
                if (Math.abs(targetServoPosition - currentServoPosition) > STALL_POSITION_THRESHOLD ||
                        Math.abs(targetServoPosition - previousTargetServoPosition) > 0.001) { // 0.001 is a small check to see if triggers are actively changing target
                    servo.setPosition(targetServoPosition);
                    currentServoPosition = targetServoPosition;
                    lastNonStalledTargetPosition = targetServoPosition;
                    stallTimer.reset(); // Reset timer because we've actively commanded a change
                    telemetry.addData("Servo Target", "%.2f", currentServoPosition);

                } else {
                    if (stallTimer.seconds() > STALL_TIMEOUT_SECONDS) {
                        telemetry.addLine(String.format("Stall suspected at %.2f after %.1fs", currentServoPosition, STALL_TIMEOUT_SECONDS));
                        isPotentiallyStalled = true;
                    } else {
                        telemetry.addData("Servo Holding/Trying", "%.2f (Timer: %.1fs)", currentServoPosition, stallTimer.seconds());
                    }
                }
            }

            telemetry.addData("Raw Target", "%.2f", targetServoPosition); // For debugging
            telemetry.addData("Is Stalled?", isPotentiallyStalled);
            telemetry.addData("Servo Pos",servopos);
            telemetry.update();

            sleep(20); // Small delay
        }
    }
}
