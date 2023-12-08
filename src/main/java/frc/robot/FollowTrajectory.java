package frc.robot;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.trajectory.Trajectory;

public class FollowTrajectory {
    private HolonomicDriveController controller;
    private Trajectory trajectory;

    public FollowTrajectory(Trajectory trajectory) {
        this.trajectory = trajectory;
        this.controller = new HolonomicDriveController(new PIDController(1, 0, 0), new PIDController(1, 0, 0), new ProfiledPIDController(agh));
    }
}