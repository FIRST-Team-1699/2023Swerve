package frc.robot.team1699.subsystems;

import java.io.File;
import java.io.IOException;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.SwerveConstants;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class Drive {
    private SwerveDrive swerve;
    private SwerveController swerveController;
   
    public Drive() {
        try {
            this.swerve = new SwerveParser(new File(Filesystem.getDeployDirectory(), "swerve")).createSwerveDrive(Units.feetToMeters(15.1));
            this.swerveController = swerve.getSwerveController();
        } catch (IOException e) {
            System.out.print("Swerve build failed");
        }

        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
    }

    public void teleopDrive(XboxController controller) {
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
        vX *= SwerveConstants.kMaxSpeed; // scaled to 50% for testing, X velocity 
        vY *= SwerveConstants.kMaxSpeed; // scaled to 50% for testing, Y velocity
        vR *= SwerveConstants.kMaxRotationalSpeed; // rotational velocity

        // drive swerve
        swerve.drive(new Translation2d(vX, vY), vR, true, false);
    }

    /** For autonomous, manually set the module states
     * @param moduleStates
     * FL, FR, BL, BR
     */
    public void setModuleStates(SwerveModuleState[] moduleStates) {
        swerve.setModuleStates(moduleStates, false);
    }

    public void zeroOdometry() {
        swerve.resetOdometry(new Pose2d(0, 0, Rotation2d.fromDegrees(0)));
    }
}
