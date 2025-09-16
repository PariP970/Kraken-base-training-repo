package frc.robot.subsystems;

import com.studica.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;



public class SwerveSubsystem extends SubsystemBase {
    
    private final SwerveModule frontLeft = new SwerveModule(
        DriveConstants.kFrontLeftDriveMotorPort,
        DriveConstants.kFrontLeftTurningMotorPort,
        DriveConstants.kModuleDriveEncoderReversed,
        DriveConstants.kModuleTurningEncoderReversed,
        DriveConstants.kFrontLeftCANCoderId,
        DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad,
        DriveConstants.kModuleCANCoderReversed);

    private final SwerveModule frontRight = new SwerveModule(
        DriveConstants.kFrontRightDriveMotorPort,
        DriveConstants.kFrontRightTurningMotorPort,
        DriveConstants.kModuleDriveEncoderReversed,
        DriveConstants.kModuleTurningEncoderReversed,
        DriveConstants.kFrontRightCANCoderId,
        DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad,
        DriveConstants.kModuleCANCoderReversed);

    private final SwerveModule backLeft = new SwerveModule(
        DriveConstants.kBackLeftDriveMotorPort,
        DriveConstants.kBackLeftTurningMotorPort,
        DriveConstants.kModuleDriveEncoderReversed,
        DriveConstants.kModuleTurningEncoderReversed,
        DriveConstants.kBackLeftCANCoderId,
        DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad,
        DriveConstants.kModuleCANCoderReversed);

    private final SwerveModule backRight = new SwerveModule(
        DriveConstants.kBackRightDriveMotorPort,
        DriveConstants.kBackRightTurningMotorPort,
        DriveConstants.kModuleDriveEncoderReversed,
        DriveConstants.kModuleTurningEncoderReversed,
        DriveConstants.kBackRightCANCoderId,
        DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad,
        DriveConstants.kModuleCANCoderReversed);


    public final GenericEntry sb_gyro;
    public final Pigeon2 pidgey = new Pigeon2(12,"*");



    public SwerveSubsystem() {

        frontLeft.resetEncoders();
        frontRight.resetEncoders();
        backLeft.resetEncoders();
        backRight.resetEncoders();

        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception e) {
            }
        }).start();
        sb_gyro = Shuffleboard.getTab(("Driver"))
        .add("Gyro",0.0)
        .withPosition(0, 0)
        .withSize(4,3)
        .getEntry();



    }

    public Command zeroHeading() {
        System.out.println("Gyro Reset");
        return Commands.runOnce(() -> pidgey.reset()); // Returns a command to be used on button press
    }

    public ChassisSpeeds getChassisSpeeds() {
        return DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
    }

    public double getHeading() {
      // float angle = m_gyro.getPitch();
        // double dAngle = angle;
        return Math.IEEEremainder(DriveConstants.kGyroReversed ? -pidgey.getYaw(true).getValueAsDouble() : pidgey.getYaw(true).getValueAsDouble(), 360);
        // return (DriveConstants.kGyroReversed ? dAngle * -1 : dAngle);
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
        
    }
    public void periodic(){
        sb_gyro.setDouble(getHeading());
    }



    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] newModuleStates = {
            frontLeft.getModuleState(),
            frontRight.getModuleState(),
            backLeft.getModuleState(),
            backRight.getModuleState()
        };
        return newModuleStates;
    }
}
