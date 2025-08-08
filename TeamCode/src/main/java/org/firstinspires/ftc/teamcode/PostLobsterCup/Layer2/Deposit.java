package org.firstinspires.ftc.teamcode.PostLobsterCup.Layer2;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.PostLobsterCup.Layer1.Deposit.AngleArm;
import org.firstinspires.ftc.teamcode.PostLobsterCup.Layer1.Deposit.Claw;
import org.firstinspires.ftc.teamcode.PostLobsterCup.Layer1.Deposit.Slides;
import org.firstinspires.ftc.teamcode.PostLobsterCup.Layer1.Deposit.Wrist;

/**
 * Deposit Subsystem
 *
 * High-level controller that integrates:
 *  - Slides
 *  - AngleArm
 *  - Wrist
 *  - Claw
 *
 * All preset positions must be tuned before use.
 */
public class Deposit {

    /** Subsystem components */
    private final Slides slides;
    private final AngleArm angleArm;
    private final Wrist wrist;
    private final Claw claw;
    private final Telemetry telemetry;

    /** ======= TUNING CONSTANTS ======= */
    // Example positions (replace with real measurements)
    private final int SLIDES_HIGH_SCORE_POS = 0;     // High scoring extension
    private final int SLIDES_MED_SCORE_POS = 0;      // Medium scoring extension
    private final int SLIDES_LOW_SCORE_POS = 0;      // Low scoring extension

    private final double ARM_DROP_POSITION = 0.0;    // Arm angle for scoring
    private final double ARM_TRANSFER_POSITION = 0.0;

    private final double WRIST_SCORE_POSITION = 0.0; // Wrist rotation for scoring
    private final double WRIST_TRANSFER_POSITION = 0.0;

    private final double CLAW_OPEN_POS = 0.0;
    private final double CLAW_CLOSED_POS = 0.0;

    public Deposit(HardwareMap hardwareMap, Telemetry telemetry,
                   String slidesMotorName, String slidesLimitName,
                   String rightArmServoName, String leftArmServoName,
                   String wristServoName, String clawServoName) {

        this.telemetry = telemetry;

        // Initialize each component
        slides = new Slides(hardwareMap, telemetry, slidesMotorName, slidesLimitName);
        angleArm = new AngleArm(hardwareMap, telemetry, rightArmServoName, leftArmServoName);
        wrist = new Wrist(hardwareMap, telemetry, wristServoName);
        claw = new Claw(hardwareMap, telemetry, clawServoName);

        // Apply initial tuning values
        angleArm.setDropPosition(ARM_DROP_POSITION);
        angleArm.setTransferPosition(ARM_TRANSFER_POSITION);

        wrist.setTransferPosition(WRIST_TRANSFER_POSITION);
        wrist.setZeroDegreePosition(WRIST_SCORE_POSITION); // Example mapping

        claw.setOpenPosition(CLAW_OPEN_POS);
        claw.setClosedPosition(CLAW_CLOSED_POS);
    }

    /** Call this periodically in the opmode loop */
    public void update() {
        slides.update();
    }

    /** Transfer sequence: prepare to receive from intake */
    public void moveToTransfer() {
        telemetry.addLine("Deposit: Moving to TRANSFER sequence...");
        slides.transferPos();
        angleArm.moveToTransferPosition();
        wrist.moveToTransferPosition();
        claw.open();
    }

    /** Score at a given height */
    public void scoreHigh() {
        telemetry.addLine("Deposit: Scoring HIGH...");
        slides.moveToPosition(SLIDES_HIGH_SCORE_POS);
        angleArm.moveToDropPosition();
        wrist.moveToZeroDegrees(); // Example: scoring wrist orientation
        claw.open();
    }

    public void scoreMedium() {
        telemetry.addLine("Deposit: Scoring MEDIUM...");
        slides.moveToPosition(SLIDES_MED_SCORE_POS);
        angleArm.moveToDropPosition();
        wrist.moveToZeroDegrees();
        claw.open();
    }

    public void scoreLow() {
        telemetry.addLine("Deposit: Scoring LOW...");
        slides.moveToPosition(SLIDES_LOW_SCORE_POS);
        angleArm.moveToDropPosition();
        wrist.moveToZeroDegrees();
        claw.open();
    }

    /** Retract everything to safe travel state */
    public void retract() {
        telemetry.addLine("Deposit: Retracting...");
        claw.close();
        wrist.moveToTransferPosition();
        angleArm.moveToTransferPosition();
        slides.retractOrInitPos();
    }

    /** Getters for manual control if needed */
    public Slides getSlides() { return slides; }
    public AngleArm getAngleArm() { return angleArm; }
    public Wrist getWrist() { return wrist; }
    public Claw getClaw() { return claw; }
}
