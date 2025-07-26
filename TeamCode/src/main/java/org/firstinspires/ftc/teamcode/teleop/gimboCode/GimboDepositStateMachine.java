package org.firstinspires.ftc.teamcode.teleop.gimboCode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.teleop.LobsterCup.Deposit;
import org.firstinspires.ftc.teamcode.teleop.LobsterCup.Intake;

/**
 * State‑machine that re‑uses the **Deposit** subsystem for claw + slide logic
 * instead of directly touching those devices. Sequence:
 *   1. Close claw (via Deposit).
 *   2. Rotate turret to deposit heading.
 *   3. Retract slides (via Deposit PID).
 */
@TeleOp(name = "GimboDepositStateMachine", group = "Combined")
public class GimboDepositStateMachine extends OpMode {

    /*** Sub‑systems ***/
    private final Intake intake = new Intake(); // wraps claw + slides + lift + wrist
    private DcMotor turretMotor;                   // turret is still controlled here

    /*** Dashboard ***/
    private FtcDashboard dashboard;

    /*** Tuning constants ***/
    private static final int    TURRET_DEPOSIT_TICKS = 800; // TODO: adjust
    public  static final double kP_TURRET           = 0.01;

    /*** Helpers ***/
    private final ElapsedTime timer = new ElapsedTime();

    /*** State machine ***/
    private enum IntakeState {
        IDLE,
        CLOSE_CLAW,
        WAIT_FOR_CLAW,
        LIFT_ARM,
        ROTATE_TURRET,
        WAIT_FOR_TURRET,
        RETRACT_SLIDES,
        DONE
    }
    private IntakeState currentState = IntakeState.IDLE;

    @Override
    public void init() {
        // Initialise sub‑systems
        intake.init(hardwareMap);
        turretMotor = hardwareMap.get(DcMotor.class, "turretMotor");
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        dashboard  = FtcDashboard.getInstance();
        telemetry  = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        telemetry.addLine("Initialized – press (A) to start deposit sequence");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Always update PID on Deposit slides
        intake.update();

        // Edge‑trigger start
        if (gamepad1.a && currentState == IntakeState.IDLE) {
            currentState = IntakeState.CLOSE_CLAW;
            timer.reset();
        }

        switch (currentState) {
            case IDLE:

                break;
            case CLOSE_CLAW:
                intake.setClawOpen(false);       // close
                currentState = IntakeState.WAIT_FOR_CLAW;
                timer.reset();
                break;

            case WAIT_FOR_CLAW:
                if (timer.milliseconds() > 300) {
                    currentState = IntakeState.ROTATE_TURRET;
                    timer.reset();
                }
                break;

            case LIFT_ARM:
                // intake.heightServo
                break;

            case ROTATE_TURRET:
                // rotate turret servo
                currentState = IntakeState.WAIT_FOR_TURRET;
                timer.reset();
                break;

            case WAIT_FOR_TURRET:
                if (timer.milliseconds() > 200) {
                    currentState = IntakeState.RETRACT_SLIDES;
                    timer.reset();
                }
                break;

            case RETRACT_SLIDES:
                intake.setSlidesTargetInches(0);  // PID will pull slides home
                timer.reset();
                // simple time‑based completion (tune as needed or expose getter in Deposit)
                if (timer.seconds() > 1.0) {
                    currentState = IntakeState.DONE;
                }
                break;

            case DONE:
                telemetry.addLine("Deposit sequence complete");
                telemetry.update();
                if (gamepad1.b) {
                    turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    currentState = IntakeState.IDLE;
                }
                break;
        }

        /** Telemetry **/
        telemetry.addData("State", currentState);
        intake.telemetry(telemetry);
        telemetry.update();
    }

    @Override
    public void stop() {
        turretMotor.setPower(0);
        // Deposit's motors will already be at zero (BRAKE)
    }
}
