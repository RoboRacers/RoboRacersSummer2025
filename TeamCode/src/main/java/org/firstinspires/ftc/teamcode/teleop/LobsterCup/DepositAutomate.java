package org.firstinspires.ftc.teamcode.teleop.LobsterCup;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DepositAutomate {
    public DcMotor verticalSlides;
    public Servo liftServoLeft;
    public Servo liftServoRight;
    public Servo wristServo;
    public Servo clawServo;

    public double targetInches = 0;
    public double kP = 0.0068, kI = 0.000007, kD = 0.00005;
    public double integralSum = 0, lastError = 0;
    private long lastTime = System.nanoTime();

    enum Score {
        IDLE, DROP, FLIP, RETRACTING
    }
    public double filteredDerivative = 0;


    enum SpecimenTransferToBar {
        IDLE, CLOSE, EXTEND, FLIP
    }


    private Score score =   Score.IDLE;
    private SpecimenTransferToBar specimenTransferToBar = SpecimenTransferToBar.IDLE;

    enum BasketTransferToBar {
        IDLE, CLOSE, EXTEND, FLIP
    }
    private BasketTransferToBar basketTransferToBar = BasketTransferToBar.IDLE;


    public void init(HardwareMap hardwareMap) {
        verticalSlides = hardwareMap.get(DcMotor.class, "depositSlide");
        liftServoLeft  = hardwareMap.get(Servo.class, "flipLeft");
        liftServoRight = hardwareMap.get(Servo.class, "flipRight");
        wristServo     = hardwareMap.get(Servo.class, "wrist");
        clawServo      = hardwareMap.get(Servo.class, "depositClaw"); // Shared with Intake

        verticalSlides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        verticalSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalSlides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        verticalSlides.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setSlidesTargetInches(double inches) {
        targetInches = inches;

        verticalSlides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        verticalSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalSlides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        verticalSlides.setDirection(DcMotorSimple.Direction.REVERSE);


        verticalSlides.setTargetPosition((int)targetInches );


        verticalSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        verticalSlides.setPower(1);
    }



    public void setSlidesPower(double power) {
        verticalSlides.setPower(power);
    }


    public void moveLift(double position) {
        position = Math.max(0.0, Math.min(1.0, position));
        liftServoLeft.setPosition(position);
        liftServoRight.setPosition(position);
    }

    public void moveWrist(double position) {
        wristServo.setPosition(Math.max(0.0, Math.min(1.0, position)));
    }

    public void setClawOpen(boolean open) {
        clawServo.setPosition(open ? 1.0 : 0.3);
    }

    private double inchesToTicks(double inches) {
        return inches * 28.229;
    }



    // Open claw to release deposit.clawServo.setPosition(0.1);
    ElapsedTime timer = new ElapsedTime();

    public void updateScoreState(boolean trigger) {
        switch (score) {
            case IDLE:
                if (trigger){
                    score = Score.DROP;
                }
                break;
            case DROP:
                // Your logic here
                clawServo.setPosition(0.1);

                score = Score.FLIP;
                break;
            case FLIP:
                // Your logic here
                moveWrist(0.1);
                moveLift(0.95);
                score = Score.RETRACTING;
                break;
            case RETRACTING:
                // Your logic here
                setSlidesTargetInches(120);
                score = Score.IDLE;
                break;
        }
    }

    public void updateSpecimenTransferState(boolean trigger) {
        switch (specimenTransferToBar) {
            case IDLE:
                if (trigger) specimenTransferToBar = SpecimenTransferToBar.CLOSE;

                break;
            case CLOSE:
                // Close claw to grab specimen
                clawServo.setPosition(0.34);
                timer.reset();
                while (timer.seconds() < 0.2){

                }
                moveWrist(0.57);
                specimenTransferToBar = SpecimenTransferToBar.EXTEND;
                break;
            case EXTEND:
                // Extend slides to 1347 ticks for specimen height
                setSlidesTargetInches(1347);

                specimenTransferToBar = SpecimenTransferToBar.FLIP;
                break;
            case FLIP:

                // Lift to 0.1489 for specimen position
                moveLift(0.1489);
                // Flip wrist to specimen scoring position
                moveWrist(0.8578);

                specimenTransferToBar = SpecimenTransferToBar.IDLE;
                break;
        }
    }


    public void updateBasketTransferState(boolean trigger) {
        switch (basketTransferToBar) {
            case IDLE:
                if (trigger) basketTransferToBar = BasketTransferToBar.CLOSE;
                break;
            case CLOSE:
                // Close claw to hold basket item
                clawServo.setPosition(0.34);
                timer.reset();
                while (timer.seconds() < 0.2){

                }
                moveWrist(0.57);
                basketTransferToBar = BasketTransferToBar.EXTEND;
                break;
            case EXTEND:
                // Extend slides to high basket (2742 ticks)
                setSlidesTargetInches(2742);
                // Lift to 0.24 for high basket

                basketTransferToBar = BasketTransferToBar.FLIP;
                break;
            case FLIP:
                // Flip wrist to 0.8578 (same as specimen)
                moveLift(0.24);
                moveWrist(0.8578);
                // Open claw to drop into basket

                basketTransferToBar = BasketTransferToBar.IDLE;
                break;
        }
    }


    public void update() {

        if (score != Score.IDLE){
            updateScoreState(true);
        }
        if(specimenTransferToBar != SpecimenTransferToBar.IDLE){{
            updateSpecimenTransferState(true);
        }}
        if(basketTransferToBar != BasketTransferToBar.IDLE){{
            updateBasketTransferState(true);
        }}


//        double ticks = targetInches;
//        double current = verticalSlides.getCurrentPosition();
//        double error = ticks - current;
//        double dt = (System.nanoTime() - lastTime) / 1e9;
//        integralSum += error * dt;
//        double derivative = (error - lastError) / dt;
//        double power = kP * error + kI * integralSum + kD * derivative;
//        power = Math.max(-1, Math.min(1, power));
//        verticalSlides.setPower(power);
//        lastError = error;
//        lastTime = System.nanoTime();
    }

    public void telemetry(Telemetry telemetry) {

        telemetry.addData("Vertical Slides Power", verticalSlides.getPower());
        telemetry.addData("Vertical Slides Pos", verticalSlides.getCurrentPosition());
        telemetry.addData("Lift Pos L", liftServoLeft.getPosition());
        telemetry.addData("Lift Pos R", liftServoRight.getPosition());
        telemetry.addData("Wrist Pos", wristServo.getPosition());
        telemetry.addData("Claw Pos", clawServo.getPosition());
    }
}













