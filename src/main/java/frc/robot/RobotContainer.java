// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.



package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;


// ===== Input Devices ===== //
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.EmergencyStopMechanismsCmd;


// ===== Swerve Specific ===== //
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.SwerveSubsystem;


// ===== Constants ===== //
import frc.robot.Constants.OIConstants;



public class RobotContainer {

    // Subsystems
    private final SwerveSubsystem swerveSubsystem;

    // Control Inputs
    private final Joystick driverJoystick = new Joystick(OIConstants.kOperatorControllerPort);
    private final Joystick translateStick = new Joystick(OIConstants.kDriverTranslateStickPort);
    private final Joystick rotateStick = new Joystick(OIConstants.kDriverRotateStickPort);



    public RobotContainer() {

        swerveSubsystem = new SwerveSubsystem();


        swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
            swerveSubsystem,
            () -> -translateStick.getRawAxis(OIConstants.kDriverYAxis),
            () -> -translateStick.getRawAxis(OIConstants.kDriverXAxis),
            () -> rotateStick.getRawAxis(0),
            () -> translateStick.getRawButton(1),
            () -> rotateStick.getRawButton(1)));


        configureBindings();
    }

    

    private void configureBindings() {

        new JoystickButton(driverJoystick, OIConstants.kDriverResetGyroButtonId).onTrue(swerveSubsystem.zeroHeading());
        // new JoystickButton(driverJoystick, OIConstants.kDriverStopButtonId).onTrue(new EmergencyStopMechanismsCmd());            MUST HAVE INCASE OF EMERGENCY
    }



    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
    
}

