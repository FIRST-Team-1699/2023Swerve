package frc.robot;

import java.io.File;
import java.io.IOException;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.SwerveConstants;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class Drive {
    private SwerveDrive swerve;
    private XboxController driveController;
    public Drive(XboxController controller) {
        try {
            this.swerve = new SwerveParser(new File(Filesystem.getDeployDirectory(), "swerve")).createSwerveDrive(Units.feetToMeters(15.1));
        } catch (IOException e) {
            System.out.print("Swerve build failed");
        }
        this.driveController = controller;
        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
    }

    public void teleopDrive() {
        // get controller inputs
        double vX = -driveController.getLeftX();
        double vY = -driveController.getLeftY();
        double vR = -driveController.getRightX();
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
        vX *= SwerveConstants.kMaxSpeed * .50; // scaled to 50% for testing, X velocity 
        vY *= SwerveConstants.kMaxSpeed * .50; // scaled to 50% for testing, Y velocity
        vR *= SwerveConstants.kMaxRotationalSpeed * .50; // rotational velocity

        // drive swerve
        swerve.drive(new Translation2d(vX, vY), vR, false, false);
    }

    /** Manually set the module states
     * @param moduleStates
     * FL, FR, BL, BR
     */
    public void setModuleStates(SwerveModuleState[] moduleStates) {
        swerve.setModuleStates(moduleStates, false);
    }

    /** Set an X to keep the swerve from moving */
    public void lock() {
        SwerveModuleState[] moduleStates = {
            new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(-45))
        }
        setModuleStates(moduleStates);
    }
}
