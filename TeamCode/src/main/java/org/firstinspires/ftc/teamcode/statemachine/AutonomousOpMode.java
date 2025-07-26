package org.firstinspires.ftc.teamcode.statemachine;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

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

        DcMotor vslideMotor = hardwareMap.get(DcMotor.class, "slide");
        DcMotor hslideMotor = hardwareMap.get(DcMotor.class, "slide");

        Servo clawServo = hardwareMap.get(Servo.class, "claw");
        Servo armServo = hardwareMap.get(Servo.class, "arm");
        Servo turretServo = hardwareMap.get(Servo.class, "turret");
        Servo depositClawServo = hardwareMap.get(Servo.class, "depositClaw");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        Drive drive = new Drive(leftFront, rightFront, leftBack, rightBack);
        Slides vslide = new Slides(vslideMotor);
        Slides hslide = new Slides(hslideMotor);
        Claw claw = new Claw(clawServo);
        DepositClaw depositClaw = new DepositClaw(depositClawServo);
        Arm arm = new Arm(armServo);
        Turret turret = new Turret(turretServo);

        StateMachines fsm = new StateMachines(drive, vslide, hslide, claw, arm, turret, depositClaw);

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


