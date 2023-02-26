// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.GripperSystem;

public class GripperToDistance extends CommandBase {
  
  GripperSystem gripperSystem;
  double goal;

  public GripperToDistance(double goal, GripperSystem gripperSystem) {
    this.goal = goal;
    this.gripperSystem = gripperSystem;
    addRequirements(gripperSystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putBoolean("grip < goal?", gripperSystem.getEncoder() < goal);
    SmartDashboard.putBoolean("grip > goal?", gripperSystem.getEncoder() > goal);
    if(gripperSystem.getEncoder() > goal - 0.2 && gripperSystem.getEncoder() < goal + 0.2){
      if(gripperSystem.getEncoder() < goal){
        gripperSystem.move(0.3);
      }else if(gripperSystem.getEncoder() > goal){
        gripperSystem.move(-0.3);
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return gripperSystem.getEncoder() > goal - 0.2 && gripperSystem.getEncoder() < goal + 0.2;
  }
}
