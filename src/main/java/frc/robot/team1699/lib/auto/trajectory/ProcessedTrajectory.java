package frc.robot.team1699.lib.auto.trajectory;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;

public class ProcessedTrajectory {
    private Trajectory trajectory;

    public ProcessedTrajectory(Path trajPath) {
        try {
            this.trajectory = TrajectoryUtil.fromPathweaverJson(trajPath);
        } catch (IOException e) {
            DriverStation.reportWarning("Trajectory failed to load", true);
        }
    }

    public Trajectory getTrajectory() {
        return this.trajectory;
    }
}
