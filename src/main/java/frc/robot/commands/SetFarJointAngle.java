// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class SetFarJointAngle extends CommandBase {
  
  double goal;
  ArmSubsystem armSubsystem;
  double D;

  public SetFarJointAngle(double goal, ArmSubsystem armSubsystem) {
    this.goal = goal;
    this.armSubsystem = armSubsystem;
    addRequirements(armSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if(!(armSubsystem.getFarJoint() > goal - 5 && armSubsystem.getFarJoint() < goal + 5)){
      if(armSubsystem.getFarJoint() < goal){
        D = 0.05 * (goal - armSubsystem.getFarJoint());
        armSubsystem.moveFarJoint(0.1 * D);
      }else if(armSubsystem.getFarJoint() > goal){
        D = 0.5 * (armSubsystem.getFarJoint() - goal);
        armSubsystem.moveFarJoint(-0.1 * D);
      }
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
