package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;


public class SwerveModule extends SubsystemBase{

    private final TalonFX driveMotor;
    private final TalonFXConfiguration driveMotorConfig = new TalonFXConfiguration();

    private final TalonFX turningMotor;
    private final TalonFXConfiguration turningMotorConfig = new TalonFXConfiguration();

    private final PIDController turningPidController;

    private final CANcoder absoluteEncoder;
    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetRad;
    
    public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
            int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {

                this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
                this.absoluteEncoderReversed = absoluteEncoderReversed;
                absoluteEncoder = new CANcoder(absoluteEncoderId, "*");
                
                System.out.println("Creating Drive Controller, ID " + driveMotorId);
                driveMotor = new TalonFX(driveMotorId,"*");
                System.out.println("Creating Turning Controller, ID " + turningMotorId);
                turningMotor = new TalonFX(turningMotorId,"*");
        
                driveMotorConfig.MotorOutput.Inverted = driveMotorReversed ? InvertedValue.CounterClockwise_Positive : InvertedValue.Clockwise_Positive;
                driveMotorConfig.Feedback.SensorToMechanismRatio = 1 / ModuleConstants.kDriveEncoderRot2Meter;
                driveMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

                turningMotorConfig.MotorOutput.Inverted = turningMotorReversed ? InvertedValue.CounterClockwise_Positive : InvertedValue.Clockwise_Positive;
                turningMotorConfig.Feedback.SensorToMechanismRatio = 1 / ModuleConstants.kTurningEncoderRot2Rad;
                turningMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

                turningPidController = new PIDController(ModuleConstants.kPTurning, 0, 0);
                turningPidController.enableContinuousInput(-Math.PI, Math.PI);
                
               
        
                driveMotor.getConfigurator().apply(driveMotorConfig);
                turningMotor.getConfigurator().apply(turningMotorConfig);
    }

    public double getDrivePosition() {
        return driveMotor.getPosition().getValue().magnitude();
    }

    public double getTurningPosition() {
        return turningMotor.getPosition().getValue().magnitude();
    }

    public double getDriveVelocity() {
        return driveMotor.getVelocity().getValue().magnitude();
    }

    public double getTurningVelocity() {
        return turningMotor.getVelocity().getValue().magnitude();
    }

    public double getAbsoluteEncoderRad() {
        double angle = absoluteEncoder.getPosition().getValue().magnitude();
        angle *= 2.0 * Math.PI;
        angle -= absoluteEncoderOffsetRad;
        return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
    }

    public void resetEncoders() {
        driveMotor.setPosition(0);
        turningMotor.setPosition(getAbsoluteEncoderRad());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        state.optimize(getState().angle);
        driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        turningMotor.set(turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));
    }

    public SwerveModuleState getModuleState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            getDrivePosition(), //* 0.9280754722679998, // modified to be more accurate * 0.9280754722679998
            new Rotation2d(getAbsoluteEncoderRad())
        );
    }

    public void stop() {
        driveMotor.set(0);
        turningMotor.set(0);
    }
}
