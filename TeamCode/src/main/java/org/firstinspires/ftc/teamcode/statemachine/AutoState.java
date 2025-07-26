package org.firstinspires.ftc.teamcode.statemachine;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

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
          sm.runtime.reset();
      }

      @Override
      public void update(StateMachines sm, LinearOpMode opMode){
          sm.vslides.setPower(0.1);
      }
    },

    VSLIDE_RETRACT {
        @Override
        public void onEnter(StateMachines sm, LinearOpMode opMode){
            opMode.telemetry.addData("State", "VSLIDE_RETRACT");
            opMode.telemetry.update();
            sm.runtime.reset();
        }

        @Override
        public void update(StateMachines sm, LinearOpMode opMode){
            sm.vslides.setPower(-0.1);
        }
    },

    HSLIDE_EXTEND {
        @Override
        public void onEnter(StateMachines sm, LinearOpMode opMode){
            opMode.telemetry.addData("State", "HSLIDE_EXTEND");
            opMode.telemetry.update();
            sm.runtime.reset();
        }

        @Override
        public void update(StateMachines sm, LinearOpMode opMode){
            sm.hslides.setPower(0.1);
        }
    },

    HSLIDE_RETRACT {
        @Override
        public void onEnter(StateMachines sm, LinearOpMode opMode){
            opMode.telemetry.addData("State", "HSLIDE_RETRACT");
            opMode.telemetry.update();
            sm.runtime.reset();
        }

        @Override
        public void update(StateMachines sm, LinearOpMode opMode){
            sm.hslides.setPower(-0.1);
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
            // claw opening code
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
            // claw closing code
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
            // arm motor code
        }
    },

    TURRET_TURN {
        @Override
        public void onEnter(StateMachines sm, LinearOpMode opMode){
            opMode.telemetry.addData("State", "TURRET_TURN");
            opMode.telemetry.update();
            sm.runtime.reset();
        }

        @Override
        public void update(StateMachines sm, LinearOpMode opMode) {
            // turret turn code
        }
    },

    ROLLING_DEPOSIT {
        @Override
        public void onEnter(StateMachines sm, LinearOpMode opMode){
            opMode.telemetry.addData("State", "ROLLING_DEPOSIT");
            opMode.telemetry.update();
            sm.runtime.reset();
        }

        @Override
        public void update(StateMachines sm, LinearOpMode opMode){
            // rolling outtake code
        }
    };


    public void onEnter(StateMachines sm, LinearOpMode opMode) {}
    public abstract void update(StateMachines sm, LinearOpMode opMode);
}
