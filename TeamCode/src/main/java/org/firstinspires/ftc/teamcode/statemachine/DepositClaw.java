package org.firstinspires.ftc.teamcode.statemachine;

import com.qualcomm.robotcore.hardware.Servo;

public class DepositClaw {
    private Servo depositClaw;
    private double depositClawPower = 0.5;
    private int depositClawOpen = 1;
    private int depositClawClose = 0;
    public DepositClaw(Servo claw) {
        this.depositClaw = claw;
    }
    public void open(){
        depositClaw.setPosition(1.0);
    }
    public void close(){
        depositClaw.setPosition(0.3);
    }
}
