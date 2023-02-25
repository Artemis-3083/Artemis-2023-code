// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.GripperSystem;

public class CloseGripper extends CommandBase {
  
  GripperSystem collectorSystem;

  public CloseGripper(GripperSystem collectorSystem) {
    this.collectorSystem = collectorSystem;
    addRequirements(collectorSystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    collectorSystem.close();
  }

  @Override
  public void end(boolean interrupted) {
    collectorSystem.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}