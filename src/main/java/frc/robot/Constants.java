package frc.robot;

import edu.wpi.first.math.util.Units;

public class Constants {
    public static class ControllerConstants {
        public static final int kDriverPort = 0;
    }

    public static class SwerveConstants {
        public static final double kDeadband = .15;
        public static final double kMaxSpeed = Units.feetToMeters(20);
        public static final double kMaxRotationalSpeed = 10;
    }
}
