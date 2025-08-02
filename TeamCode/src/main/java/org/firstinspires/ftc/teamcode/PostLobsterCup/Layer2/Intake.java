package org.firstinspires.ftc.teamcode.PostLobsterCup.Layer2;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.PostLobsterCup.Layer1.Intake.*;
import org.firstinspires.ftc.teamcode.PostLobsterCup.Layer1.Intake.Vision.CombinedHSVandAnglePipeline;
import org.firstinspires.ftc.teamcode.PostLobsterCup.Layer1.Intake.Vision.PixelToDistanceMapper;
import org.firstinspires.ftc.teamcode.PostLobsterCup.Layer1.Intake.Vision.VisionSystem;

public class Intake {

    private final Claw claw;
    private final Wrist wrist;
    private final Forebar forebar;
    private final Turret turret;
    private final Slides slides;
    public VisionSystem visionSystem;



    public Intake(HardwareMap hardwareMap, Telemetry telemetry, CombinedHSVandAnglePipeline.TargetColor targetColor) {
        claw = new Claw(hardwareMap, telemetry, "clawServo");
        wrist = new Wrist(hardwareMap, telemetry, "rotateServo");
        forebar = new Forebar(hardwareMap, telemetry, "heightServo");
        turret = new Turret(hardwareMap, telemetry, "turret");
        slides = new Slides(hardwareMap, telemetry, "intakeSlide", "limitSwitch");
        visionSystem = new VisionSystem(hardwareMap, telemetry, targetColor);
    }

    /** All components to pickup state */
    public void readyForPickSubmersible() {
        claw.open();
        turret.moveToSubmersibleZonePosition();
        sleep(200);
        wrist.moveToZeroDegrees();  // 0°
        forebar.moveToSubmersibleZonePosition();
        slides.retractOrInitPos();
    }

    /** Closes claw and lifts slightly */
    public void grabAndTransferPosIntake() {
        claw.close();
        sleep(250); // Wait for grip
        forebar.moveToTransferPosition();
        turret.moveToTransferPosition();
        wrist.moveToNinetyDegrees();  // 90°
        slides.transferPos();
    }


    /** All components to submersible zone state */
    public void goUnderSubZone() {
        wrist.moveToZeroDegrees();
        forebar.moveToSubmersibleZonePosition();
        turret.moveToSubmersibleZonePosition();
        slides.retractOrInitPos();
    }

    public void intakeInit(){
        claw.open();
        wrist.moveToZeroDegrees();
        forebar.moveToTransferPosition();
        turret.moveToInitPos();
        slides.retractOrInitPos();
    }

    public void moveIntakeSlidesInches(double slidesInchPos){
        slides.moveInches(slidesInchPos);
    }

    public void moveTurretAngle(double angle){
        turret.moveToAngle(angle);
    }

    public void moveWristAngle(double angle){
        wrist.moveToAngle(angle);
    }

    public void moveVisionIntakeSystem(double slidesInches, double turretAngle, double wristAngle){
        slides.moveInches(slidesInches);
        sleep(200);
        wrist.moveToAngle(wristAngle);
        turret.moveToAngle(turretAngle);
    }

    public void autoPickupFromSnapshot() throws InterruptedException {
        visionSystem.triggerSnapshot();
        visionSystem.triggerSnapshot();
        if (!visionSystem.hasProcessedSnapshot()) {
            return;
        }
        readyForPickSubmersible();
        PixelToDistanceMapper.DistanceResult result = visionSystem.getMappedResult();
        double wristAngle = visionSystem.getSampleAngle();
        double distanceInches = result.forwardDist;
        double slidesDistance = distanceInches - Math.sqrt(81- (Math.pow(result.horizOffset,2)));
        double turretAngle = Math.acos(result.horizOffset/9);
        moveVisionIntakeSystem(slidesDistance, turretAngle, wristAngle);
        forebar.moveToPickPosition();
        grabAndTransferPosIntake();
    }

    /** Call this every loop to monitor limit switch */
    public void update() {
        slides.update();  // Check and reset encoder if needed
    }

    /** Blocking delay */
    private void sleep(int millis) {
        try {
            Thread.sleep(millis);
        } catch (InterruptedException ignored) {}
    }
}
