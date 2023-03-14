// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSystem;

public class AlwaysPID extends CommandBase {
  
  ArmSubsystem armSubsystem;
  ElevatorSystem elevatorSystem;
  double elevatorGoal;
  double closeJointGoal;
  double farJointGoal;

  public AlwaysPID(ArmSubsystem armSubsystem, ElevatorSystem elevatorSystem) {
    this.elevatorSystem = elevatorSystem;
    this.armSubsystem = armSubsystem;
    elevatorGoal = 0;
    closeJointGoal = 0;
    farJointGoal = 0;
    addRequirements(elevatorSystem, armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    new ElevatorPID(elevatorGoal, elevatorSystem).alongWith(new ArmPID(closeJointGoal, farJointGoal, armSubsystem)).execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public void setElevatorGoal(double elevatorGoal){
    this.elevatorGoal = elevatorGoal;
  }

  public void setCloseJointGoal(double closeJointGoal){
    this.closeJointGoal = closeJointGoal;
  }

  public void setFarJointGoal(double farJointGoal){
    this.farJointGoal = farJointGoal;
  }
}
