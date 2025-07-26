package org.firstinspires.ftc.teamcode.statemachine;

import com.qualcomm.robotcore.hardware.DcMotor;

// The Drive class is a simple abstraction layer to control a two-motor drive system.
public class Slides {
    // These are the motor references for the left and right drive motors.
    private DcMotor slide;

    public static int targetPosEncoderTicks = 0;
    public final int extendedPos = 0;
    public final int retractedPos = 0;


    // Constructor: Initializes the left and right motors with the ones passed in.
    public Slides(DcMotor slide) {
        this.slide = slide;
    }

    // Sets both motors to the same power to move the robot forward or backward.
    public void setPower(double power) {
        slide.setPower(power);   // Left motor set to given power
    }

    // Sets motors to opposite powers to make the robot turn in place.
    // This causes the robot to spin in place (clockwise if power > 0)


    // Stops both motors by setting power to 0.
    public void stop() {
        slide.setPower(0);
    }
    public static int getTargetPosEncoderTicks(){
        return targetPosEncoderTicks;
    }
    public static void setTargetPosEncoderTicks(int targetPosEncoderTicks){
        Slides.targetPosEncoderTicks = targetPosEncoderTicks;
    }
    public void goToTargetPos(){
        slide.setTargetPosition(targetPosEncoderTicks);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide.setPower(1);
    }
    public void extend(){
        slide.setTargetPosition(extendedPos);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide.setPower(1);
    }
    public void retract(){
        slide.setTargetPosition(retractedPos);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide.setPower(-1);
    }
    public DcMotor getSlide(){
        return slide;
    }
}
