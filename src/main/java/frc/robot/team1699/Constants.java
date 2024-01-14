package frc.robot.team1699;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public class Constants {
    public static class ControllerConstants {
        public static final int kDriverPort = 0;
    }

    public static class SwerveConstants {
        public static final double kDeadband = .15;
        public static final double kMaxSpeed = Units.feetToMeters(15.1);
        public static final double kMaxRotationalSpeed = 15; // DOES THIS NEED TO BE DIFFERENT THAN MAX SPEED?
        public static final double kTrackWidthDiv2 = Units.inchesToMeters(10.25);
        public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            new Translation2d(kTrackWidthDiv2, kTrackWidthDiv2),
            new Translation2d(-kTrackWidthDiv2, kTrackWidthDiv2),
            new Translation2d(kTrackWidthDiv2, -kTrackWidthDiv2),
            new Translation2d(-kTrackWidthDiv2, -kTrackWidthDiv2)
        );
    }
}
