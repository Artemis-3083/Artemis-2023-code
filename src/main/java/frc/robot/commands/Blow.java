// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.GripperSystem;


public class Blow extends CommandBase {
  
  GripperSystem gripperSystem;

  public Blow(GripperSystem gripperSystem) {
    this.gripperSystem = gripperSystem;
    addRequirements(gripperSystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    gripperSystem.moveWheels(1);
  }

  @Override
  public void end(boolean interrupted) {
    gripperSystem.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}