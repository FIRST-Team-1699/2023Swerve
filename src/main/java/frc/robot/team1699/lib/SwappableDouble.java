package frc.robot.team1699.lib;

import java.util.ArrayList;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwappableDouble {
    private String name;
    private final double defaultValue;
    private double value;

    public static ArrayList<SwappableDouble> values = new ArrayList<SwappableDouble>();

    public static void update() {
        for(SwappableDouble value : values) {
            value.updateValue();
        }
    }

    public SwappableDouble(String name, double value) {
        this.name = name;
        this.defaultValue = value;
        this.value = value;
        SmartDashboard.putNumber(name, value);
        values.add(this);
    }

    public void setValue(double value) {
        this.value = value;
        sendValue();
    }

    public void sendValue() {
        SmartDashboard.putNumber(name, value);
    }

    public void updateValue() {
        this.value = SmartDashboard.getNumber(name, defaultValue);
    }

    public double getValue() {
        return value;
    }
}
