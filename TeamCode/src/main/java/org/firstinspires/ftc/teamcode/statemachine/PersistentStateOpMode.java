package org.firstinspires.ftc.teamcode.statemachine;

import android.content.ContentValues;
import android.content.Context;
import android.database.Cursor;
import android.database.sqlite.SQLiteDatabase;
import android.database.sqlite.SQLiteOpenHelper;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.function.DoubleUnaryOperator;

@TeleOp(name = "Persistent SQLite OpMode", group = "Concept")
public class PersistentStateOpMode extends LinearOpMode {

    // === Enum to define persistent keys ===
    enum Key {
        BATTERY_LEVEL,
        MOTOR_POWER,
        POSE_X,
        POSE_Y,
        HEADING
    }

    // === SQLite helper + persistent state logic all in one ===
    static class StateStore {
        private final SQLiteDatabase db;

        public StateStore(Context context) {
            SQLiteOpenHelper helper = new SQLiteOpenHelper(context, "robot_state.db", null, 1) {
                @Override
                public void onCreate(SQLiteDatabase db) {
                    db.execSQL("CREATE TABLE IF NOT EXISTS robot_state (`key` TEXT PRIMARY KEY, value TEXT)");
                }

                @Override
                public void onUpgrade(SQLiteDatabase db, int oldVersion, int newVersion) {
                    db.execSQL("DROP TABLE IF EXISTS robot_state");
                    onCreate(db);
                }
            };
            this.db = helper.getWritableDatabase();
        }

        public void set(Key key, double value) {
            ContentValues values = new ContentValues();
            values.put("key", key.name());
            values.put("value", String.valueOf(value));
            db.insertWithOnConflict("robot_state", null, values, SQLiteDatabase.CONFLICT_REPLACE);
        }

        public double get(Key key) {
            Cursor cursor = db.rawQuery("SELECT value FROM robot_state WHERE `key` = ?", new String[]{key.name()});
            double value = 0.0;
            if (cursor.moveToFirst()) {
                value = Double.parseDouble(cursor.getString(0));
            }
            cursor.close();
            return value;
        }

        public void update(Key key, DoubleUnaryOperator updateFn) {
            double newVal = updateFn.applyAsDouble(get(key));
            set(key, newVal);
        }

        public void close() {
            db.close();
        }
    }

    // === FTC OpMode Logic ===
    @Override
    public void runOpMode() {
        StateStore store = new StateStore(hardwareMap.appContext);

        // Set initial values (if needed)
        store.set(Key.BATTERY_LEVEL, 100.0);
        store.set(Key.MOTOR_POWER, 0.8);

        telemetry.addLine("Initialized persistent state");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            double battery = store.get(Key.BATTERY_LEVEL);
            telemetry.addData("Battery", battery);
            telemetry.addData("Motor Power", store.get(Key.MOTOR_POWER));

            // Simulate battery drain
            store.update(Key.BATTERY_LEVEL, val -> val - 0.05);

            telemetry.update();
            sleep(100);
        }

        store.close();
    }
}
