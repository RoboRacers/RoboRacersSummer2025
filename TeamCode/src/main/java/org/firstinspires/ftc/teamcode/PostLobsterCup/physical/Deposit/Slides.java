package org.firstinspires.ftc.teamcode.PostLobsterCup.physical.Deposit;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Deposit Slides Control
 *
 * This class controls the deposit slides for extending/retracting the scoring mechanism.
 * All constants MUST be measured and tuned before competition use.
 */
public class Slides {

    private DcMotor slidesMotor;
    private DigitalChannel limitSwitch;
    private Telemetry telemetry;

    /** ======= TUNING CONSTANTS (PLACEHOLDERS) ======= */
    // Encoder positions (in ticks) for each key state
    private final int MIN_POSITION = 0;          // Fully retracted
    private final int MAX_POSITION = 0;          // Fully extended
    private final int TRANSFER_POS = 0;          // Transfer position for intake → deposit handoff
    private final int RETRACTED_OR_INIT_POS = 0; // Starting position

    // Conversion from inches of travel to encoder ticks
    private final double TICKS_PER_INCH = 0; // Measure pulley diameter & gear ratio

    // Motor power for movement (0.0 to 1.0)
    private final double SLIDES_POWER = 1.0;

    /** ======= INTERNAL STATE ======= */
    private boolean wasLimitPreviouslyPressed = false;

    public Slides(HardwareMap hardwareMap, Telemetry telemetry, String motorName, String limitSwitchName) {
        slidesMotor = hardwareMap.get(DcMotor.class, motorName);
        limitSwitch = hardwareMap.get(DigitalChannel.class, limitSwitchName);
        this.telemetry = telemetry;

        // Configure hardware
        limitSwitch.setMode(DigitalChannel.Mode.INPUT);
        slidesMotor.setDirection(DcMotor.Direction.REVERSE); // Change if needed
        slidesMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidesMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slidesMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /** Call this in the loop to update encoder reset & telemetry */
    public void update() {
        boolean isLimitPressed = !limitSwitch.getState(); // FALSE = pressed

        // Reset encoder when switch is first pressed
        if (isLimitPressed && !wasLimitPreviouslyPressed) {
            slidesMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slidesMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            telemetry.addLine("Deposit Slides: Limit switch pressed → Encoder reset");
        }

        wasLimitPreviouslyPressed = isLimitPressed;

        telemetry.addData("Deposit Slides Encoder", slidesMotor.getCurrentPosition());
        telemetry.addData("Deposit Slides Limit Switch", isLimitPressed ? "PRESSED" : "NOT PRESSED");
        telemetry.update();
    }

    /** Moves slides to a target encoder position */
    public void moveToPosition(int targetPosition) {
        targetPosition = Math.max(MIN_POSITION, Math.min(MAX_POSITION, targetPosition));

        slidesMotor.setTargetPosition(targetPosition);
        slidesMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slidesMotor.setPower(SLIDES_POWER);

        telemetry.addData("Deposit Slides", "Moving to position: %d", targetPosition);
        telemetry.update();
    }

    /** Moves to the retracted / init position */
    public void retractOrInitPos() {
        moveToPosition(RETRACTED_OR_INIT_POS);
        telemetry.addData("Deposit Slides", "Moving to INIT position: %d", RETRACTED_OR_INIT_POS);
        telemetry.update();
    }

    /** Moves a relative distance in inches */
    public void moveInches(double inches) {
        int targetTicks = (int) (inches * TICKS_PER_INCH);
        moveToPosition(getCurrentPosition() + targetTicks);
        telemetry.addData("Deposit Slides", "Moving %.2f inches → %d ticks", inches, targetTicks);
        telemetry.update();
    }

    /** Moves to transfer position for scoring handoff */
    public void transferPos() {
        moveToPosition(TRANSFER_POS);
        telemetry.addData("Deposit Slides", "Moving to TRANSFER position: %d", TRANSFER_POS);
        telemetry.update();
    }

    /** Stops slides immediately */
    public void stop() {
        slidesMotor.setPower(0);
        slidesMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addData("Deposit Slides", "Motor stopped");
        telemetry.update();
    }

    /** Returns current encoder position */
    public int getCurrentPosition() {
        return slidesMotor.getCurrentPosition();
    }

    /** Returns whether the limit switch is pressed */
    public boolean isLimitPressed() {
        return !limitSwitch.getState(); // false = pressed
    }
}