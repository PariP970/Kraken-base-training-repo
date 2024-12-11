// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;    
import edu.wpi.first.wpilibj2.command.Commands;   
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GriffinSubsystem extends SubsystemBase {

  private final CANSparkMax motor;

  public GriffinSubsystem() {
    motor = new CANSparkMax(0, MotorType.kBrushless);
  }

  public Command motorOnCmd() {                   
    return Commands.runOnce(() -> motorOn());     
  }                                               

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void motorOn() {                  
    motor.set(1);                     
  }                                         
}

