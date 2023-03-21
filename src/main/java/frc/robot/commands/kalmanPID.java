// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSystem;

public class kalmanPID extends CommandBase {

  PIDController pid;
  DriveSystem driveSystem;
  Double calculate;

  public kalmanPID(DriveSystem driveSystem) {
    this.driveSystem = driveSystem;
    pid = new PIDController(0.2, 0.1, 0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(driveSystem.getPitch() > 0) {
     calculate = pid.calculate(driveSystem.kalmanEstametion(),0);
     MathUtil.clamp(calculate, 0, 0.7);
     driveSystem.drive(calculate, 0, 0);
    }
    else {
      calculate = pid.calculate(driveSystem.kalmanEstametion(),0);
      MathUtil.clamp(calculate, 0, 0.7);
      driveSystem.drive(0, calculate, 0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
