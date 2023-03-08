// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSystem;

public class KalmanBalance extends CommandBase {
  
  DriveSystem driveSystem;
  double calculate;
  PIDController pidController;

  public KalmanBalance(DriveSystem driveSystem) {
    this.driveSystem = driveSystem;
    pidController = new PIDController(0.1, 0, 0.3);
    addRequirements(driveSystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    calculate = pidController.calculate(driveSystem.getPitch(), 0);
    if(!(driveSystem.getPitch() > -1 && driveSystem.getPitch() < 1)){
      if(driveSystem.getPitch() > 0){
        // calculate = MathUtil.clamp(calculate, -0.7, 0);
        driveSystem.drive(-calculate, 0, 0);
      }else if(driveSystem.getPitch() < 0){
        // calculate = MathUtil.clamp(calculate, 0, 0.7);
        driveSystem.drive(0, calculate, 0);
      }  
    }else{
      driveSystem.stop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
