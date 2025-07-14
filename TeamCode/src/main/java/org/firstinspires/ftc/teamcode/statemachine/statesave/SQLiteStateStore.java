package org.firstinspires.ftc.teamcode.statemachine.statesave;

import android.content.ContentValues;
import android.content.Context;
import android.database.Cursor;
import android.database.sqlite.SQLiteDatabase;

import java.util.HashMap;
import java.util.Map;
import java.util.function.DoubleUnaryOperator;

public class SQLiteStateStore {
    private final SQLiteDatabase db;

    public enum RobotStateKey {
        BATTERY_LEVEL,
        MOTOR_POWER,
        POSE_X,
        POSE_Y,
        HEADING
    }

    private final Map<RobotStateKey, StateRule<Double>> ruleMap = new HashMap<>();

    // Constructor
    public SQLiteStateStore(Context context) {
        RobotStateDBHelper helper = new RobotStateDBHelper(context);
        db = helper.getWritableDatabase();
    }

    public void setRule(RobotStateKey key, StateRule<Double> rule) {
        ruleMap.put(key, rule);
    }

    public void set(RobotStateKey key, double value) {
        StateRule<Double> rule = ruleMap.get(key);
        if (rule != null && !rule.isValid(value)) {
            return; // Don't update if invalid
        }

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

    public boolean contains(RobotStateKey key) {
        Cursor cursor = db.rawQuery("SELECT 1 FROM robot_state WHERE state_key = ?", new String[]{key.name()});
        boolean exists = cursor.moveToFirst();
        cursor.close();
        return exists;
    }

    public void update(RobotStateKey key, DoubleUnaryOperator updateFn) {
        double current = get(key);
        double updated = updateFn.applyAsDouble(current);

        // Re-apply rule here to enforce on updates
        StateRule<Double> rule = ruleMap.get(key);
        if (rule != null && !rule.isValid(updated)) {
            return; // Skip invalid update
        }

        set(key, updated);
    }

    public void resetDatabase() {
        db.execSQL("DROP TABLE IF EXISTS robot_state");
        db.execSQL("CREATE TABLE robot_state (state_key TEXT PRIMARY KEY, value TEXT)");
    }

    public void close() {
        db.close();
    }
}
