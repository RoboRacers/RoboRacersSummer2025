package org.firstinspires.ftc.teamcode.statemachine.statesave;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "Persistent TeleOp", group = "Examples")
public class PersistentOpMode extends LinearOpMode {
    private SQLiteStateStore stateStore;
    private List<String> batteryHistory = new ArrayList<>();

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

        // Subscribe to battery changes and record them
        stateStore.subscribe(SQLiteStateStore.RobotStateKey.BATTERY_LEVEL, (key, newValue) -> {
            batteryHistory.add("Battery changed: " + newValue);
        });

        waitForStart();

        while (opModeIsActive()) {
            double battery = stateStore.get(SQLiteStateStore.RobotStateKey.BATTERY_LEVEL);
            telemetry.addData("Current Battery", battery);

            // Only reduce battery if above 0
            if (battery > 0) {
                stateStore.update(SQLiteStateStore.RobotStateKey.BATTERY_LEVEL, level -> level - 0.02);
            }

            // Reset the database if X is pressed
            if (gamepad1.x) {
                stateStore.resetDatabase();
                batteryHistory.clear();
                telemetry.addLine("Database reset!");
            }

            // Show battery change history
            for (String message : batteryHistory) {
                telemetry.addLine(message);
            }

            telemetry.update();
            sleep(100);
        }

        stateStore.close();
    }
}
