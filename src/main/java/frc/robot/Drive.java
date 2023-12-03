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

public class Drive {
    private SwerveDrive swerve;

    public Drive() {
        try {
            this.swerve = new SwerveParser(new File(Filesystem.getDeployDirectory(), "swerve")).createSwerveDrive(Units.feetToMeters(15.1));
        } catch (IOException e) {
            System.out.print("Swerve build failed");
        }
    }

    public void teleopDrive(XboxController controller) {
        // get controller inputs
        double vX = controller.getLeftX();
        double vY = -controller.getLeftY();
        double vR = controller.getRightX();
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
        vX = controller.getLeftX() * SwerveConstants.kMaxSpeed * .50; // scaled to 20% for testing, X velocity 
        vY = -controller.getLeftY() * SwerveConstants.kMaxSpeed * .50; // scaled to 20% for testing, Y velocity
        vR *= -1;
        vR *= SwerveConstants.kMaxRotationalSpeed * .50; // rotational velocity
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
}
