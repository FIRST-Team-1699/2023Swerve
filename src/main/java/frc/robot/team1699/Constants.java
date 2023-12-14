package frc.robot.team1699;

import edu.wpi.first.math.util.Units;

public class Constants {
    public static class ControllerConstants {
        public static final int kDriverPort = 0;
    }

    public static class SwerveConstants {
        public static final double kDeadband = .15;
        public static final double kMaxVelocity = Units.feetToMeters(15.1);
        public static final double kMaxAngularVelocity = 10;
    }
}
