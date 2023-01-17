// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSystem;

public class ElevatorUp extends CommandBase {
  /** Creates a new ElevatorUp. */
  public ElevatorSystem elevator;
  public ElevatorUp(ElevatorSystem elevator) {
    this.elevator = elevator;
    addRequirements(elevator);
  }

  @Override
  public void initialize() {
    elevator.elevatorUp();
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    elevator.stop();
  }

  @Override
  public boolean isFinished() {
    return elevator.getHeight()>=Constants.ELEVATOR_HEIGHT;
  }
}
