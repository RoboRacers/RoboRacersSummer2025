package org.firstinspires.ftc.teamcode.PostLobsterCup.Layer1.Intake;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Slides {

    private DcMotor slidesMotor;
    private DigitalChannel limitSwitch;
    private Telemetry telemetry;

    private final int MIN_POSITION = 0;
    private final int MAX_POSITION = 550;
    private final int TRANSFER_POS = 260;
    private final int RETRACTED_OR_INIT_POS = 0;

    private final double TICKS_PER_INCH = 28.0;

    // Prevent multiple resets on repeated press
    private boolean wasLimitPreviouslyPressed = false;

    public Slides(HardwareMap hardwareMap, Telemetry telemetry, String motorName, String limitSwitchName) {
        slidesMotor = hardwareMap.get(DcMotor.class, motorName);
        limitSwitch = hardwareMap.get(DigitalChannel.class, limitSwitchName);
        this.telemetry = telemetry;

        limitSwitch.setMode(DigitalChannel.Mode.INPUT);

        slidesMotor.setDirection(DcMotor.Direction.REVERSE);
        slidesMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void update() {
        boolean isLimitPressed = !limitSwitch.getState(); // FALSE = PRESSED

        if (isLimitPressed && !wasLimitPreviouslyPressed) {
            // First frame it's pressed — reset encoder
            slidesMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            telemetry.addLine("Limit switch PRESSED: Encoder reset");
        }

        wasLimitPreviouslyPressed = isLimitPressed;

        telemetry.addData("Slides Encoder", slidesMotor.getCurrentPosition());
        telemetry.addData("Limit Switch", isLimitPressed ? "PRESSED" : "NOT PRESSED");
        telemetry.update();
    }

    public void moveToPosition(int targetPosition) {
        targetPosition = Math.max(MIN_POSITION, Math.min(MAX_POSITION, targetPosition));

        slidesMotor.setTargetPosition(targetPosition);
        slidesMotor.setPower(1.0);

        telemetry.addData("Intake Slides", "Moving to position: %d", targetPosition);
        telemetry.update();
    }

    public void retractOrInitPos() {
        moveToPosition(RETRACTED_OR_INIT_POS);
        telemetry.addData("Intake Slides", "Moving to INIT position: %d", RETRACTED_OR_INIT_POS);
        telemetry.update();
    }

    public void moveInches(double inches) {
        int targetTicks = (int) (inches * TICKS_PER_INCH);
        moveToPosition(targetTicks);
        telemetry.addData("Intake Slides", "Moving %.2f inches → %d ticks", inches, targetTicks);
        telemetry.update();
    }

    public void transferPos() {
        moveToPosition(TRANSFER_POS);
        telemetry.addData("Intake Slides", "Moving to TRANSFER position: %d", TRANSFER_POS);
        telemetry.update();
    }

    public void stop() {
        slidesMotor.setPower(0);
        telemetry.addData("Intake Slides", "Motor stopped");
        telemetry.update();
    }

    public int getCurrentPosition() {
        return slidesMotor.getCurrentPosition();
    }

    public boolean isLimitPressed() {
        return !limitSwitch.getState(); // false = PRESSED
    }
}
