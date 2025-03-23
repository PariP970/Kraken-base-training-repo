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
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

import com.ctre.phoenix6.hardware.CANcoder;


public class SwerveModule extends SubsystemBase{

    private final SparkMax driveMotor;
    private final SparkMaxConfig driveMotorConfig = new SparkMaxConfig();

    private final SparkMax turningMotor;
    private final SparkMaxConfig turningMotorConfig = new SparkMaxConfig();

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turningEncoder;

    private final PIDController turningPidController;

    private final CANcoder absoluteEncoder;
    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetRad;
    
    public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
            int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {

                this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
                this.absoluteEncoderReversed = absoluteEncoderReversed;
                absoluteEncoder = new CANcoder(absoluteEncoderId, "rio");
                
                System.out.println("Creating Drive Controller, ID " + driveMotorId);
                driveMotor = new SparkMax(driveMotorId, MotorType.kBrushless);
                System.out.println("Creating Turning Controller, ID " + turningMotorId);
                turningMotor = new SparkMax(turningMotorId, MotorType.kBrushless);
        
                driveMotorConfig
                    .inverted(driveMotorReversed)
                    .idleMode(IdleMode.kBrake);
                driveMotorConfig.encoder
                    .positionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter)
                    .velocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);
        
                turningMotorConfig
                    .inverted(turningMotorReversed);
                turningMotorConfig.encoder
                    .positionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad)
                    .velocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);
        
                turningPidController = new PIDController(ModuleConstants.kPTurning, 0, 0);
                turningPidController.enableContinuousInput(-Math.PI, Math.PI);
                
                driveEncoder = driveMotor.getEncoder();
                turningEncoder = turningMotor.getEncoder();
        
                driveMotor.configure(driveMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
                turningMotor.configure(turningMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
                resetEncoders();
    }

    public double getDrivePosition() {
        return driveEncoder.getPosition();
    }

    public double getTurningPosition() {
        return turningEncoder.getPosition();
    }

    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }

    public double getTurningVelocity() {
        return turningEncoder.getVelocity();
    }

    public double getAbsoluteEncoderRad() {
        double angle = absoluteEncoder.getPosition().getValue().magnitude();
        angle *= 2.0 * Math.PI;
        angle -= absoluteEncoderOffsetRad;
        return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
    }

    public void resetEncoders() {
        driveEncoder.setPosition(0);
        turningEncoder.setPosition(getAbsoluteEncoderRad());
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

    public void stop() {
        driveMotor.set(0);
        turningMotor.set(0);
    }
}
