package org.firstinspires.ftc.teamcode.statemachine;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

//import org.firstinspires.ftc.robotcore.external.StateMachine;
//import org.firstinspires.ftc.robotcore.external.StateMachine;
import org.firstinspires.ftc.teamcode.statemachine.AutoState;
import org.firstinspires.ftc.teamcode.statemachine.Drive;
import org.firstinspires.ftc.teamcode.statemachine.StateMachines;

@TeleOp(name = "Enum FSM Auto")
public class AutonomousOpMode extends LinearOpMode {

    @Override
    public void runOpMode() {
        DcMotor leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        DcMotor rightFront = hardwareMap.get(DcMotor.class, "rightFront");

        DcMotor leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        DcMotor rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        Drive drive = new Drive(leftFront, rightFront);
        StateMachines fsm = new StateMachines(drive);

        waitForStart();

        fsm.setState(AutoState.IDLE, this);

        while (opModeIsActive()) {
            if (gamepad1.square){
                fsm.setState(AutoState.MOVE_FORWARD, this);
            }
            if (gamepad1.cross){
                fsm.setState(AutoState.TURN, this);
            }
            if (gamepad1.triangle){
                fsm.setState(AutoState.STOP, this);
            }
            fsm.update(this);
            sleep(20);  // Allow cooperative multitasking
        }
    }
}


