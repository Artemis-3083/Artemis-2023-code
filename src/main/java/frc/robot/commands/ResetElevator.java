// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSystem;

public class ResetElevator extends CommandBase {
  
  ElevatorSystem elevatorSystem;

  public ResetElevator(ElevatorSystem elevatorSystem) {
    this.elevatorSystem = elevatorSystem;
    addRequirements(elevatorSystem);  
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!elevatorSystem.getLimitSwitch()){
      elevatorSystem.move(0.5);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevatorSystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    SmartDashboard.putBoolean("finished elevator reset", elevatorSystem.getLimitSwitch());
    return elevatorSystem.getLimitSwitch();
  }
}
