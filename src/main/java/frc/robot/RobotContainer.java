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
    private final Joystick controller = new Joystick(OIConstants.kOperatorControllerPort);



    public RobotContainer() {

        swerveSubsystem = new SwerveSubsystem();


        swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
            swerveSubsystem,
            () -> -controller.getRawAxis(OIConstants.kDriverYAxis),
            () -> -controller.getRawAxis(OIConstants.kDriverXAxis),
            () -> controller.getRawAxis(OIConstants.kDriverRotAxis),
            () -> controller.getRawButton(OIConstants.kController_rightBumper),
            () -> controller.getRawButton(OIConstants.kController_leftBumper)));


        configureBindings();
    }

    

    private void configureBindings() {

        new JoystickButton(controller, OIConstants.kDriverResetGyroButtonId).onTrue(swerveSubsystem.zeroHeading());
        // new JoystickButton(controller, OIConstants.kDriverStopButtonId).onTrue(new EmergencyStopMechanismsCmd());
    }



    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
    
}

