package frc.robot.team1699.lib.auto.events;

import edu.wpi.first.math.trajectory.Trajectory;
import frc.robot.team1699.subsystems.Drive;
import frc.robot.team1699.subsystems.Drive.DriveState;

public class FollowTrajectory extends Event {
    private Trajectory trajectory;
    private Drive swerve;

    public FollowTrajectory(Trajectory trajectory, Drive swerve) {
        this.trajectory = trajectory;
        this.swerve = swerve;
    }

    public void initialize() {
        swerve.setTrajectory(trajectory);
        swerve.setWantedState(DriveState.FOLLOW_TRAJ);
    }

    public void update() {}
    
    public boolean isFinished() {
        if(swerve.doneWithTraj() == true) {
            return true;
        }
        return false;
    }

    public void finish() {
        swerve.setWantedState(DriveState.LOCK);
    }
}
