package frc.robot;

import java.io.File;
import java.io.IOException;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.SwerveConstants;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class Drive {
    private DriveState currentState = DriveState.LOCK;
    private DriveState wantedState = DriveState.LOCK;

    private Timer trajTimer = new Timer();
    private Trajectory trajectory;
    private HolonomicDriveController driveController;

    private SwerveDrive swerve;
    private XboxController controller;
    public Drive(XboxController controller) {
        try {
            this.swerve = new SwerveParser(new File(Filesystem.getDeployDirectory(), "swerve")).createSwerveDrive(Units.feetToMeters(15.1));
        } catch (IOException e) {
            System.out.print("Swerve build failed");
        }
        this.controller = controller;
        this.driveController = new HolonomicDriveController(new PIDController(.7, 0, 0), new PIDController(.7, 0, 0), new ProfiledPIDController(.5, 0, 0, new TrapezoidProfile.Constraints(4, 3)));
        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
    }

    private void teleopDrive() {
        // get controller inputs
        double vX = -controller.getLeftX();
        double vY = -controller.getLeftY();
        double vR = -controller.getRightX();
        // apply deadbands
        if(Math.abs(vX) < SwerveConstants.kDeadband) {
            vX = 0;
        }
        if(Math.abs(vY) < SwerveConstants.kDeadband) {
            vY = 0;
        }
        if(Math.abs(vR) < SwerveConstants.kDeadband) {
            vR = 0;
        }
        // scale outputs
        vX *= SwerveConstants.kMaxSpeed; 
        vY *= SwerveConstants.kMaxSpeed;
        vR *= SwerveConstants.kMaxRotationalSpeed;

        // drive swerve
        swerve.drive(new Translation2d(vX, vY), vR, false, false);
    }

    public void setTrajectory(Trajectory trajectory) {
        this.trajectory = trajectory;
    }

    // /** Manually set the module states
    //  * @param moduleStates
    //  * FL, FR, BL, BR
    //  */
    // private void setModuleStates(SwerveModuleState[] moduleStates) {
    //     swerve.setModuleStates(moduleStates, false);
    // }

    /** Set an X to keep the swerve from moving */
    private void lock() {
        swerve.lockPose();
    }

    public void update() {
        switch (currentState) {
            case FOLLOW_TRAJ:
                // if(trajTimer.get() < trajectory.getTotalTimeSeconds()) {
                //     Trajectory.State targetState = trajectory.sample(trajTimer.get());
                //     ChassisSpeeds targetSpeeds = driveController.calculate(swerve.getPose(), targetState, Rotation2d.fromDegrees(0));
                //     swerve.drive(targetSpeeds);
                // } else {
                //     trajTimer.stop();
                // }
                break;
            case LOCK:
                lock();
                break;
            case TELEOP_DRIVE:
                teleopDrive();
                break;
            default:
                break;
        }
    }

    private void handleStateTransition() {
        switch (wantedState) {
            case FOLLOW_TRAJ:
                trajTimer.restart();
                break;
            case LOCK:
                break;
            case TELEOP_DRIVE:
                break;
            default:
                break;

        }
    }

    public void setState(DriveState state) {
        if(this.wantedState != state) {
            wantedState = state;
            handleStateTransition();
        }
    }

    public DriveState getState() {
        return this.currentState;
    }

    /** For auto
     * @param states
     * FL, FR, BL, BR
     */
    public void setModuleStates(SwerveModuleState[] states) {
        swerve.setModuleStates(states, false);
    }

    public Pose2d getPose() {
        return swerve.getPose();
    }

    public enum DriveState {
        TELEOP_DRIVE,
        LOCK,
        FOLLOW_TRAJ
    }
}
