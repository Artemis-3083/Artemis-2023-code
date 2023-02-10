// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSystem;

public class SetElevatorHeight extends CommandBase {

  public ElevatorSystem elevator;
  public double height;

  /** Creates a new SetElevatorHeight. */
  public SetElevatorHeight(double height,ElevatorSystem elevator) {
    this.elevator=elevator;
    this.height=height;
    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(elevator.getHeight()>height)
      elevator.elevatorDown();
    else if(elevator.getHeight()<height)
      elevator.elevatorUp();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevator.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return elevator.getHeight()==height;
  }
}
