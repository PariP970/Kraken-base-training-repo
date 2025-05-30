package frc.robot;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public class Constants {
    
    public static final class ModuleConstants {
        public static final double kWheelDiameterMeters = Units.inchesToMeters(0);
        public static final double kDriveMotorGearRatio = 1.0 / 1.0; // Drive ratio of ? : 1
        public static final double kTurningMotorGearRatio = 1.0 / (1.0 / 1.0); // Turning ratio of (? / ?) : 1
        public static final double kDriveEncoderRot2Meter = Math.PI * kWheelDiameterMeters / kDriveMotorGearRatio;
        public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60.0;
        public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60.0;
        public static final double kPTurning = 0.5; // For PID
    }

    public static final class DriveConstants {

        public static final double kTrackWidth = Units.inchesToMeters(0); // Distance between right and left wheels
        public static final double kWheelBase = Units.inchesToMeters(0); // Distance between front and back wheels

        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, kTrackWidth / 2), // back left
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2), // back right
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2), // front left
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)); // front right

        // DRIVE Motor Ports
        public static final int kFrontLeftDriveMotorPort = 0;
        public static final int kBackLeftDriveMotorPort = 0;
        public static final int kFrontRightDriveMotorPort = 0;
        public static final int kBackRightDriveMotorPort = 0;

        // TURNING Motor Ports
        public static final int kFrontLeftTurningMotorPort = 0;
        public static final int kBackLeftTurningMotorPort = 0;
        public static final int kFrontRightTurningMotorPort = 0;
        public static final int kBackRightTurningMotorPort = 0;

        // CANCoder Ids
        public static final int kFrontLeftCANCoderId = 0;
        public static final int kBackLeftCANCoderId = 0;
        public static final int kFrontRightCANCoderId = 0;
        public static final int kBackRightCANCoderId = 0;

        // Invert booleans | We use MK4i modules so the turning motors are inverted
        public static final boolean kModuleTurningEncoderReversed = true;
        public static final boolean kModuleDriveEncoderReversed = false;
        public static final boolean kModuleCANCoderReversed = false;
        public static final boolean kGyroReversed = true;

        // Turning encoder offsets
        public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = 0 * Math.PI / 180;
        public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = 0 * Math.PI / 180;
        public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = 0 * Math.PI / 180;
        public static final double kBackRightDriveAbsoluteEncoderOffsetRad = 0 * Math.PI / 180;

        // Robot speeds
        public static final double kPhysicalMaxSpeedMetersPerSecond = 1; // PHYSICAL max speed of the modules (safety cap)
        public static final double kTeleDriveMaxSpeedMetersPerSecond = 1; // Max speed set for teleop

        // Robot turning speeds
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * Math.PI;
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond / 2;

        // Robot acceleration
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;

        // Robot speed modifiers
        public static final double kTeleopBoostModifier = 1.5;
        public static final double kTeleopSlowModifier = 0.5;
    }

    public static final class OIConstants {
        // Ports
        public static final int kOperatorControllerPort = 0;
        public static final int kDriverTranslateStickPort = 1;
        public static final int kDriverRotateStickPort = 2;
        public static final double kDeadband = 0.05;

        // Joysticks
        public static final int kDriverYAxis = 1;
        public static final int kDriverXAxis = 0;
        public static final int kDriverRotAxis = 2;

        // Joystick Triggers
        public static final int kDriverBoostButtonId = 1;

        // Buttons
        public static final int kDriverResetGyroButtonId = 2;
        public static final int kDriverStopButtonId = 10;

        // Buttons
        public static final int kController_x = 1;
        public static final int kController_a = 2;
        public static final int kController_b = 3;
        public static final int kController_y = 4;
        
        public static final int kController_back = 9;
        public static final int kController_start = 10;
        public static final int kController_leftStickButton = 11;
        public static final int kController_rightStickButton = 12;
        
        // Triggers
        public static final int kController_leftBumper = 5;
        public static final int kController_rightBumper = 6;
        public static final int kController_leftTrigger = 7;
        public static final int kController_rightTrigger = 8;
    } 
}
