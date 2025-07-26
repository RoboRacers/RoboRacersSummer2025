package org.firstinspires.ftc.teamcode.statemachine;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

//import org.firstinspires.ftc.robotcore.external.StateMachines;
//import org.firstinspires.ftc.teamcode.StateMachines.StateMachiness;
import org.firstinspires.ftc.teamcode.statemachine.StateMachines;

public enum AutoState {
    //Set first state here
    IDLE {
        @Override
        public void onEnter(StateMachines sm, LinearOpMode opMode) {
            opMode.telemetry.addData("State", "IDLE");
            opMode.telemetry.update();
        }

        @Override
        public void update(StateMachines sm, LinearOpMode opMode) {
            sm.setState(MOVE_FORWARD, opMode);
        }
    },
    MOVE_FORWARD {
        @Override
        public void onEnter(StateMachines sm, LinearOpMode opMode) {
            opMode.telemetry.addData("State", "MOVE_FORWARD");
            opMode.telemetry.update();
            sm.runtime.reset();
            sm.drive.setPower(0.5);
        }

        @Override
        public void update(StateMachines sm, LinearOpMode opMode) {
            if (sm.runtime.seconds() > 2.0) {
                sm.drive.setPower(0);
            }
        }
    },
    TURN {
        @Override
        public void onEnter(StateMachines sm, LinearOpMode opMode) {
            opMode.telemetry.addData("State", "TURN");
            opMode.telemetry.update();
            sm.runtime.reset();
            sm.drive.turn(0.3);
        }

        @Override
        public void update(StateMachines sm, LinearOpMode opMode) {
            if (sm.runtime.seconds() > 1.5) {
                sm.drive.stop();
            }
        }
    },
    STOP {
        @Override
        public void onEnter(StateMachines sm, LinearOpMode opMode) {
            opMode.telemetry.addData("State", "STOP");
            opMode.telemetry.update();
            sm.drive.stop();
        }

        @Override
        public void update(StateMachines sm, LinearOpMode opMode) {
            // Remain here
        }
    },
    VSLIDE_EXTEND {
      @Override
      public void onEnter(StateMachines sm, LinearOpMode opMode){
          opMode.telemetry.addData("State", "VSLIDE_EXTEND");
          opMode.telemetry.update();
          sm.vslides.getSlide().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
          sm.vslides.getSlide().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
          sm.vslides.getSlide().setDirection(DcMotorSimple.Direction.REVERSE);
          sm.runtime.reset();
      }

      @Override
      public void update(StateMachines sm, LinearOpMode opMode){
          sm.vslides.extend();
          opMode.telemetry.addData("Current Pos", sm.vslides.getSlide().getCurrentPosition());
          opMode.telemetry.addData("Target Pos", sm.vslides.getSlide().getTargetPosition());
          opMode.telemetry.update();
      }
    },
    VSLIDE_RETRACT {
        @Override
        public void onEnter(StateMachines sm, LinearOpMode opMode){
            opMode.telemetry.addData("State", "VSLIDE_RETRACT");
            opMode.telemetry.update();
            sm.vslides.getSlide().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            sm.vslides.getSlide().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            sm.vslides.getSlide().setDirection(DcMotorSimple.Direction.REVERSE);
            sm.runtime.reset();
        }

        @Override
        public void update(StateMachines sm, LinearOpMode opMode){
            sm.vslides.retract();
        }
    },
    HSLIDE_EXTEND {
        @Override
        public void onEnter(StateMachines sm, LinearOpMode opMode){
            opMode.telemetry.addData("State", "HSLIDE_EXTEND");
            opMode.telemetry.update();
            sm.hslides.getSlide().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            sm.hslides.getSlide().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            sm.hslides.getSlide().setDirection(DcMotorSimple.Direction.REVERSE);
            sm.runtime.reset();
        }

        @Override
        public void update(StateMachines sm, LinearOpMode opMode){
            sm.hslides.extend();
        }
    },
    HSLIDE_RETRACT {
        @Override
        public void onEnter(StateMachines sm, LinearOpMode opMode){
            opMode.telemetry.addData("State", "HSLIDE_RETRACT");
            opMode.telemetry.update();
            sm.hslides.getSlide().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            sm.hslides.getSlide().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            sm.hslides.getSlide().setDirection(DcMotorSimple.Direction.REVERSE);
            sm.runtime.reset();
        }

        @Override
        public void update(StateMachines sm, LinearOpMode opMode){
            sm.hslides.retract();
        }
    },
    CLAW_OPEN {
        @Override
        public void onEnter(StateMachines sm, LinearOpMode opMode){
            opMode.telemetry.addData("State", "CLAW_OPEN");
            opMode.telemetry.update();
            sm.runtime.reset();

        }
        @Override
        public void update(StateMachines sm, LinearOpMode opMode) {
            sm.claw.open();
        }
    },
    CLAW_CLOSE {
        @Override
        public void onEnter(StateMachines sm, LinearOpMode opMode){
            opMode.telemetry.addData("State", "CLAW_CLOSE");
            opMode.telemetry.update();
            sm.runtime.reset();
        }

        @Override
        public void update(StateMachines sm, LinearOpMode opMode){
            sm.claw.close();
        }
    },
    ARM_FLIP {
        @Override
        public void onEnter(StateMachines sm, LinearOpMode opMode){
            opMode.telemetry.addData("State", "ARM_FLIP");
            opMode.telemetry.update();
            sm.runtime.reset();
        }

        @Override
        public void update(StateMachines sm, LinearOpMode opMode){
            sm.arm.flip();
        }
    },
    ARM_UNFLIP {
        @Override
        public void onEnter(StateMachines sm, LinearOpMode opMode){
            opMode.telemetry.addData("State", "ARM_UNFLIP");
            opMode.telemetry.update();
            sm.runtime.reset();
        }

        @Override
        public void update(StateMachines sm, LinearOpMode opMode){
            sm.arm.unflip();
        }
    },
    TURRET_TURN_FORWARD {
        @Override
        public void onEnter(StateMachines sm, LinearOpMode opMode){
            opMode.telemetry.addData("State", "TURRET_TURN_FORWARD");
            opMode.telemetry.update();
            sm.runtime.reset();
        }

        @Override
        public void update(StateMachines sm, LinearOpMode opMode) {
            sm.turret.forward();
        }
    },
    TURRET_TURN_BACKWARD {
        @Override
        public void onEnter(StateMachines sm, LinearOpMode opMode){
            opMode.telemetry.addData("State", "TURRET_TURN_BACKWARD");
            opMode.telemetry.update();
            sm.runtime.reset();
        }

        @Override
        public void update(StateMachines sm, LinearOpMode opMode) {
            sm.turret.back();
        }
    },
    DEPOSIT_CLAW_OPEN {
        @Override
        public void onEnter(StateMachines sm, LinearOpMode opMode){
            opMode.telemetry.addData("State", "ROLLING_DEPOSIT");
            opMode.telemetry.update();
            sm.runtime.reset();
        }

        @Override
        public void update(StateMachines sm, LinearOpMode opMode){
            sm.depositClaw.open();
        }
    },
    DEPOSIT_CLAW_CLOSE {
        @Override
        public void onEnter(StateMachines sm, LinearOpMode opMode){
            opMode.telemetry.addData("State", "ROLLING_DEPOSIT");
            opMode.telemetry.update();
            sm.runtime.reset();
        }

        @Override
        public void update(StateMachines sm, LinearOpMode opMode){
            sm.depositClaw.close();
        }
    };
    public void onEnter(StateMachines sm, LinearOpMode opMode) {}
    public abstract void update(StateMachines sm, LinearOpMode opMode);
}
