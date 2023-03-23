// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSystem;

public class ElevatorDown extends CommandBase {

  public ArmSubsystem armSubsystem;
  public ElevatorSystem elevatorSystem;
  
  public ElevatorDown(ElevatorSystem elevator, ArmSubsystem armSubsystem) {
    this.elevatorSystem = elevator;
    this.armSubsystem = armSubsystem;
    addRequirements(elevator, armSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if(!elevatorSystem.isAtRiskElevator() && armSubsystem.isAtRiskArm()){
      elevatorSystem.move(-0.4);
    }
  }

  @Override
  public void end(boolean interrupted) {
    elevatorSystem.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
