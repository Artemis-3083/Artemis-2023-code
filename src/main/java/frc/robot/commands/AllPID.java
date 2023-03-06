// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSystem;

public class AllPID extends CommandBase {
  
  ArmSubsystem armSubsystem;
  ElevatorSystem elevatorSystem;
  PIDController closeController;
  PIDController farController;
  PIDController elevatorController; 
  double closeGoal;
  double farGoal;
  double elevatorGoal;
  double closeCalcuation;
  double farCalcuation;
  double elevatorCalcuation;
  
  public AllPID(ArmSubsystem armSubsystem, ElevatorSystem elevatorSystem, double closeGoal, double farGoal, double elevatorGoal) {
    closeController = new PIDController(0.09, 0, 0);
    farController = new PIDController(0.13, 0, 0);
    elevatorController = new PIDController(0.1, 0, 0);
    this.elevatorSystem = elevatorSystem;
    this.armSubsystem = armSubsystem;
    this.farGoal = farGoal;
    this.closeGoal = closeGoal;
    this.elevatorGoal = elevatorGoal;
    addRequirements(elevatorSystem, armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elevatorCalcuation = elevatorController.calculate(elevatorSystem.getHeight(), elevatorGoal);
    closeCalcuation = closeController.calculate(armSubsystem.getCloseJoint(), closeGoal);
    farCalcuation = farController.calculate(armSubsystem.getFarJoint(), farGoal);

    if(closeCalcuation < 0){
      closeCalcuation = MathUtil.clamp(closeCalcuation, -0.22, 0);
    }else if(closeCalcuation > 0){
      closeCalcuation = MathUtil.clamp(closeCalcuation, 0, 0.22);
    }

    if(farCalcuation < 0){
      farCalcuation = MathUtil.clamp(farCalcuation, -0.175, 0);
    }else if(farCalcuation > 0){
      farCalcuation = MathUtil.clamp(farCalcuation, 0, 0.2);
    }

    if(elevatorCalcuation < 0){
      elevatorCalcuation = MathUtil.clamp(elevatorCalcuation, -0.3, 0);
    }else if(elevatorCalcuation > 0){
      elevatorCalcuation = MathUtil.clamp(elevatorCalcuation, 0, 0.3);
    }
    
    // if(!elevatorController.atSetpoint()){
    elevatorSystem.move(elevatorCalcuation);
    // }
    // if(!closeController.atSetpoint()){
    armSubsystem.moveCloseJoint(closeCalcuation);
    // }
    // if(!farController.atSetpoint()){
    armSubsystem.moveFarJoint(farCalcuation);
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(elevatorController.atSetpoint() && closeController.atSetpoint() && farController.atSetpoint()){
      return true;
    }
    return false;
  }
}
