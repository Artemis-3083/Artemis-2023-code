// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class ArmPID extends CommandBase {
  
  ArmSubsystem armSubsystem;
  PIDController closeController;
  PIDController farController;
  double closeGoal;
  double farGoal;
  double closeCalcuation;
  double farCalcuation;

  public ArmPID(double closeGoal, double farGoal, ArmSubsystem armSubsystem) {
    this.farGoal = farGoal;
    this.closeGoal = closeGoal;
    closeController = new PIDController(0.175, 0, 0);
    farController = new PIDController(0.15, 0, 0);
    this.armSubsystem = armSubsystem;
    addRequirements(armSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    closeCalcuation = closeController.calculate(armSubsystem.getCloseJoint(), closeGoal);
    farCalcuation = farController.calculate(armSubsystem.getFarJoint(), farGoal);

    if(closeCalcuation < 0){
      closeCalcuation = MathUtil.clamp(closeCalcuation, -0.15, 0);
    }else if(closeCalcuation > 0){
      closeCalcuation = MathUtil.clamp(closeCalcuation, 0, 0.15);
    }

    if(farCalcuation < 0){
      farCalcuation = MathUtil.clamp(farCalcuation, -0.1, 0);
    }else if(farCalcuation > 0){
      farCalcuation = MathUtil.clamp(farCalcuation, 0, 0.1);
    }

    if(!closeController.atSetpoint()){
      armSubsystem.moveCloseJoint(closeCalcuation);
    }
    if(!farController.atSetpoint()){
      armSubsystem.moveFarJoint(farCalcuation);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    armSubsystem.stopCloseJoint();
    armSubsystem.stopFarJoint();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(closeController.atSetpoint() && farController.atSetpoint()){
      return true;  
    }
    return false;
  }
}
