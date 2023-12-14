package frc.robot.team1699.lib;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwappableBoolean {
    private String name;
    private final boolean defaultValue;
    private boolean value;

    public static ArrayList<SwappableBoolean> values = new ArrayList<SwappableBoolean>();

    public static void update() {
        for(SwappableBoolean value : values) {
            value.updateValue();
        }
    }

    public SwappableBoolean(String name, boolean value) {
        this.name = name;
        this.defaultValue = value;
        this.value = value;
        SmartDashboard.putBoolean(name, value);
        values.add(this);
    }

    public void setValue(boolean value) {
        this.value = value;
        sendValue();
    }

    public void sendValue() {
        SmartDashboard.putBoolean(name, value);
    }

    public void updateValue() {
        this.value = SmartDashboard.getBoolean(name, defaultValue);
    }

    public boolean getValue() {
        return value;
    }
}
