package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public final class Constants {
    public static final class CANIDS {
        public static final class SwerveModules {
            public static final class FrontLeft {
                public static final int turnMotor = 69;
                public static final int driveMotor = 69;
                public static final int absEncoderPwmChannel = 0;
            }

            public static final class FrontRight {
                public static final int turnMotor = 69;
                public static final int driveMotor = 69;
                public static final int CANcoder = 69;
            }

            public static final class BackLeft {
                public static final int turnMotor = 69;
                public static final int driveMotor = 69;
                public static final int CANcoder = 69;
            }

            public static final class BackRight {
                public static final int turnMotor = 69;
                public static final int driveMotor = 69;
                public static final int CANcoder = 69;
            }
        }
    }

    public static final class SwerveDriveCharacteristics {
        public static final double ksAngularVolts = 0.05; //not to be confused with the reported kS from an angular diff drivetrain test
        public static final double ksLinearVolts = 0.17825;
        public static final double kvLinearVoltMeters = 2.2146;
        public static final double kaLinearVoltMeters = 0.278;
        public static final Translation2d chassisDimensionsMeters = new Translation2d(Units.inchesToMeters(22.0), Units.inchesToMeters(22.0));
    }

    public static final class DriveControllerCharacteristics {
        public static final int port = 0;
        public static final double deadband = 0.02;
    }
}
