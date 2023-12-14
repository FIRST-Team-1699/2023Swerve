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
import frc.robot.team1699.Constants.SwerveConstants;
import frc.robot.team1699.lib.SwappableBoolean;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class Drive {
    private SwerveDrive swerve;
    private SwerveController swerveController;
    private SwappableBoolean fieldRelativeDrive;
    private SwappableBoolean fieldRelativeRotation;
    private SwappableBoolean openLoopSwerve;
   
    public Drive() {
        try {
            this.swerve = new SwerveParser(new File(Filesystem.getDeployDirectory(), "swerve")).createSwerveDrive(Units.feetToMeters(15.1));
            this.swerveController = swerve.getSwerveController();
        } catch (IOException e) {
            System.out.print("Swerve build failed");
        }
        fieldRelativeDrive = new SwappableBoolean("field_relative_drive", true);
        fieldRelativeRotation = new SwappableBoolean("field_relative_rotation", false);
        openLoopSwerve = new SwappableBoolean("open_loop_swerve", false);
        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
    }

    public void teleopDrive(XboxController controller) {
        // get controller inputs
        double leftX = -controller.getLeftX();
        double leftY = -controller.getLeftY();
        double rightX = -controller.getRightX();
        double rightY = -controller.getRightY();
        // apply deadbands
        if(Math.abs(leftX) < SwerveConstants.kDeadband) {
            leftX = 0;
        }
        if(Math.abs(leftY) < SwerveConstants.kDeadband) {
            leftY = 0;
        }
        if(Math.abs(rightX) < SwerveConstants.kDeadband) {
            rightX = 0;
        }
        if(Math.abs(rightY) < SwerveConstants.kDeadband) {
            rightY = 0;
        }
        // scale outputs
        leftX *= SwerveConstants.kMaxVelocity; // scaled to 50% for testing, X velocity 
        leftY *= SwerveConstants.kMaxVelocity; // scaled to 50% for testing, Y velocity
        rightX *= SwerveConstants.kMaxAngularVelocity; // rotational velocity

        if(fieldRelativeRotation.getValue()) {
            ChassisSpeeds speeds = swerveController.getTargetSpeeds(leftX, leftY, rightX, rightY, swerve.getYaw().getRadians(), SwerveConstants.kMaxVelocity);
            Translation2d translation = SwerveController.getTranslation2d(speeds);
            swerve.drive(translation, speeds.omegaRadiansPerSecond, fieldRelativeDrive.getValue(), openLoopSwerve.getValue());
        } else {
            swerve.drive(new Translation2d(leftX, leftY), rightX, fieldRelativeDrive.getValue(), openLoopSwerve.getValue());
        }
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
