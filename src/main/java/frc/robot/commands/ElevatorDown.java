// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;

public class ElevatorDown extends CommandBase {
  /** Creates a new ElevatorDown. */
  public Elevator elevator;
  public ElevatorDown(Elevator elevator) {
    this.elevator = elevator;
    addRequirements(elevator);
  }

  @Override
  public void initialize() {
    elevator.elevatorDown();
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrDownted) {
    elevator.stop();
  }

  @Override
  public boolean isFinished() {
    return elevator.getHeight()<=0;
  }
}
