package frc.robot.team1699.lib.auto.modes;

public abstract class AutoMode {
    public abstract void initialize();

    public abstract void run();

    public abstract boolean isFinished();

    public abstract void finish();
}