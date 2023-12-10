package frc.robot;


import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.SwerveConstants;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class FollowTrajectory {
    private Trajectory trajectory;
    private Drive swerve;
    private final HolonomicDriveController driveController = new HolonomicDriveController(
        new PIDController(.7, 0, 0), new PIDController(.7, 0, 0), new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(4, 2))
    );
    private final Timer timer = new Timer();

    public FollowTrajectory(Trajectory trajectory, Drive swerve) {
        this.trajectory = trajectory;
        this.swerve = swerve;
    }

    public void start() {
        timer.start();
    }

    public void run() {
        Trajectory.State desiredState = trajectory.sample(timer.get());
        ChassisSpeeds desiredSpeeds = driveController.calculate(swerve.getPose(), 
            desiredState.poseMeters, 
            desiredState.velocityMetersPerSecond, 
            desiredState.poseMeters.getRotation());
        SwerveModuleState[] desiredStates = SwerveConstants.kinematics.toSwerveModuleStates(desiredSpeeds);
        swerve.setModuleStates(desiredStates);
    }

    public boolean isFinished() {
        if(timer.get() > trajectory.getTotalTimeSeconds()) {
            return true;
        }
        return false;
    }

    public void finish() {
        SwerveModuleState[] stoppedState = {
            new SwerveModuleState(),
            new SwerveModuleState(),
            new SwerveModuleState(),
            new SwerveModuleState()
        };
        swerve.setModuleStates(stoppedState);
    }
}
