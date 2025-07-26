package org.firstinspires.ftc.teamcode.statemachine;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw {
    private Servo claw;
    private double clawPower = 0.5;
    private int clawOpen = 1;
    private int clawClose = 0;
    public Claw(Servo claw) {
        this.claw = claw;
    }
    public void open(){
        claw.setPosition(1.0);
    }
    public void close(){
        claw.setPosition(0.3);
    }
}
