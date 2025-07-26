package org.firstinspires.ftc.teamcode.statemachine;

import com.qualcomm.robotcore.hardware.DcMotor;
public class Claw {
    private DcMotor claw;
    private double clawPower = 0.5;
    private int clawOpen = 1;
    private int clawClose = 0;
    public Claw(DcMotor claw) {
        this.claw = claw;
    }
    public void open(){
        claw.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        claw.setTargetPosition(clawOpen);
        claw.setPower(clawPower);
    }
    public void setPower(double power) {
        claw.setPower(power);   // Left motor set to given power
    }
    public void stop() {
        claw.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        claw.setTargetPosition(clawClose);
        claw.setPower(clawPower);
    }
}
