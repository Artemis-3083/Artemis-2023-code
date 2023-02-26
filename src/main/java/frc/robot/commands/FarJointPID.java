// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class FarJointPID extends CommandBase {
  
  PIDController pidController;
  ArmSubsystem armSubsystem;
  double goal;
  double calcuation;

  public FarJointPID(double goal, ArmSubsystem armSubsystem) {
    this.armSubsystem = armSubsystem;
    addRequirements(armSubsystem);
    this.goal = goal;
    pidController = new PIDController(0.15, 0, 0);
  }

  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    calcuation = pidController.calculate(armSubsystem.getFarJoint(), goal);
    if(calcuation < 0){
      calcuation = MathUtil.clamp(calcuation, -0.1, 0);
    }else if(calcuation > 0){
      calcuation = MathUtil.clamp(calcuation, 0, 0.1);
    }
    armSubsystem.moveFarJoint(calcuation);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    armSubsystem.stopFarJoint();
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
