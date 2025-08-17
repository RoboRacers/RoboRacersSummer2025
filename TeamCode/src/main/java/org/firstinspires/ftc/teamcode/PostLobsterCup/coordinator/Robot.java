package org.firstinspires.ftc.teamcode.PostLobsterCup.coordinator;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.PostLobsterCup.physical.Intake.Vision.CombinedHSVandAnglePipeline;
import org.firstinspires.ftc.teamcode.PostLobsterCup.logical.Intake;

public class Robot {

    public enum State {
        INIT,
        READY_FOR_PICK,
        SNAPSHOT,
        WAIT_FOR_SNAPSHOT,
        MOVE_TO_PICK_SAMPLE_WITH_VISION,
        GRAB_SAMPLE_AND_MOVE_INTAKE_TRANSFER,
        DONE
    }

    private final Intake intake;
    private final Telemetry telemetry;

    private State currentState = State.INIT;
    private CombinedHSVandAnglePipeline.TargetColor targetColor;


    private boolean snapshotTriggered = false;
    private boolean pickupCompleted = false;

    public Robot(HardwareMap hardwareMap, Telemetry telemetry, CombinedHSVandAnglePipeline.TargetColor targetColor) {
        this.telemetry = telemetry;
        this.intake = new Intake(hardwareMap, telemetry, targetColor);
//        this.targetColor = targetColor;
    }

    public void update() throws InterruptedException {
        intake.update();

        switch (currentState) {
            case INIT:
                intake.intakeInit();
                break;

            case READY_FOR_PICK:
                intake.readyForPickSubmersible();
                transitionTo(State.SNAPSHOT);
                break;

            case SNAPSHOT:
//                intake.visionSystem.setTargetColor(targetColor);
                intake.visionSystem.triggerSnapshot();
                snapshotTriggered = true;
                transitionTo(State.WAIT_FOR_SNAPSHOT);
                break;

            case WAIT_FOR_SNAPSHOT:
                if (intake.visionSystem.hasProcessedSnapshot()) {
                    transitionTo(State.MOVE_TO_PICK_SAMPLE_WITH_VISION);
                }
                break;

            case MOVE_TO_PICK_SAMPLE_WITH_VISION:
//                PixelToDistanceMapper.DistanceResult result = intake.visionSystem.getMappedResult();
//                double wristAngle = intake.visionSystem.getSampleAngle();
//                double distanceInches = result.forwardDist;
//                double slidesDistance = distanceInches - Math.sqrt(81 - (Math.pow(result.horizOffset, 2)));
//                double turretAngle = Math.acos(result.horizOffset / 9);
//
//                telemetry.addData("AutoPickup: Wrist Angle", wristAngle);
//                telemetry.addData("AutoPickup: Turret Angle", turretAngle);
//                telemetry.addData("AutoPickup: Slides Distance", slidesDistance);
//                telemetry.update();
//
//                intake.moveVisionIntakeSystem(slidesDistance, turretAngle, wristAngle);
//                //transitionTo(State.GRAB_SAMPLE_AND_MOVE_INTAKE_TRANSFER);
                intake.autoPickupFromSnapshot();
                break;

            case GRAB_SAMPLE_AND_MOVE_INTAKE_TRANSFER:
                intake.grabAndTransferPosIntake();
                pickupCompleted = true;
                transitionTo(State.DONE);
                break;

            case DONE:
                // Stop or wait for further instructions
                break;
        }
    }

    public void setState(State newState) {
        this.currentState = newState;
    }

    public void setTargetColor(CombinedHSVandAnglePipeline.TargetColor color) {
        this.targetColor = color;
    }

    public State getCurrentState() {
        return currentState;
    }

    public boolean isPickupCompleted() {
        return pickupCompleted;
    }

    public void shutdown() {
        intake.visionSystem.stop();
    }

    private void transitionTo(State newState) {
        currentState = newState;
    }
}
