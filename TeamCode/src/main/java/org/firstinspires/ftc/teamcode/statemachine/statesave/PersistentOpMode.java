package org.firstinspires.ftc.teamcode.statemachine.statesave;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Persistent TeleOp", group = "Examples")
public class PersistentOpMode extends LinearOpMode {
    private SQLiteStateStore stateStore;

    @Override
    public void runOpMode() {
        stateStore = new SQLiteStateStore(hardwareMap.appContext);

        // Rule: battery must be >= 0
        stateStore.setRule(SQLiteStateStore.RobotStateKey.BATTERY_LEVEL, level -> level >= 0);

        // Only set defaults if no value is saved yet
        if (!stateStore.contains(SQLiteStateStore.RobotStateKey.BATTERY_LEVEL)) {
            stateStore.set(SQLiteStateStore.RobotStateKey.BATTERY_LEVEL, 95.0);
        }

        if (!stateStore.contains(SQLiteStateStore.RobotStateKey.MOTOR_POWER)) {
            stateStore.set(SQLiteStateStore.RobotStateKey.MOTOR_POWER, 0.7);
        }

        waitForStart();

        while (opModeIsActive()) {
            double battery = stateStore.get(SQLiteStateStore.RobotStateKey.BATTERY_LEVEL);
            telemetry.addData("Battery", battery);

            // Only reduce battery if above 0
            if (battery > 0) {
                stateStore.update(SQLiteStateStore.RobotStateKey.BATTERY_LEVEL, level -> level - 0.02);
            }

            // Reset the database if X is pressed
            if (gamepad1.square) {
                stateStore.resetDatabase();
                stateStore.set(SQLiteStateStore.RobotStateKey.BATTERY_LEVEL, 95.0);
                telemetry.addLine("Database reset!");
            }

            telemetry.update();
            sleep(100);
        }

        stateStore.close();
    }
}
