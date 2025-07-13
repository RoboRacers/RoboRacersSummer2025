package org.firstinspires.ftc.teamcode.statemachine;

import android.content.ContentValues;
import android.content.Context;
import android.database.Cursor;
import android.database.sqlite.SQLiteDatabase;

import org.firstinspires.ftc.teamcode.statemachine.RobotStateDBHelper;

public class SQLiteStateStore {
    private final SQLiteDatabase db;
    public enum RobotStateKey {
        BATTERY_LEVEL,
        MOTOR_POWER,
        POSE_X,
        POSE_Y,
        HEADING
    }

    // This a constructor
    public SQLiteStateStore(Context context) {
        RobotStateDBHelper helper = new RobotStateDBHelper(context);
        db = helper.getWritableDatabase();
    }


    public void set(RobotStateKey key, double value) {
        ContentValues values = new ContentValues();
        values.put("state_key", key.name());
        values.put("value", String.valueOf(value));
        db.insertWithOnConflict("robot_state", null, values, SQLiteDatabase.CONFLICT_REPLACE);
    }

    public double get(RobotStateKey key) {
        Cursor cursor = db.rawQuery("SELECT value FROM robot_state WHERE state_key = ?", new String[]{key.name()});
        if (cursor.moveToFirst()) {
            String val = cursor.getString(0);
            cursor.close();
            return Double.parseDouble(val);
        }
        cursor.close();
        return 0.0;
    }

    public void update(RobotStateKey key, java.util.function.DoubleUnaryOperator updateFn) {
        double current = get(key);
        double updated = updateFn.applyAsDouble(current);
        set(key, updated);
    }

    public void close() {
        db.close();
    }
}
