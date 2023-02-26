// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSystem;
import frc.robot.subsystems.GripperSystem;

public class GripperPID extends CommandBase {
  
  PIDController pidController;
  GripperSystem gripperSystem;
  double goal;
  double calcuation;

  public GripperPID(double goal, GripperSystem gripperSystem) {
    this.gripperSystem = gripperSystem;
    addRequirements(gripperSystem);
    this.goal = goal;
    pidController = new PIDController(1, 0 ,0);
  }

  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    calcuation = pidController.calculate(gripperSystem.getEncoder(), goal);
    if(calcuation < 0){
      calcuation = MathUtil.clamp(calcuation, -0.3, 0);
    }else if(calcuation > 0){
      calcuation = MathUtil.clamp(calcuation, 0, 0.3);
    }
    gripperSystem.move(calcuation);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    gripperSystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(pidController.atSetpoint()){
      return true;  
    }
    return false;
  }
}
