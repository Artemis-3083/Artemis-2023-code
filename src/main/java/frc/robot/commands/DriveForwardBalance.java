// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSystem;

public class DriveForwardBalance extends CommandBase {
  
  DriveSystem driveSystem;

  public DriveForwardBalance(DriveSystem driveSystem) {
    this.driveSystem = driveSystem;
    addRequirements(driveSystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveSystem.drive(0.5, 0, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return driveSystem.getDistancePassedM() > 80000; //?
  }
}
