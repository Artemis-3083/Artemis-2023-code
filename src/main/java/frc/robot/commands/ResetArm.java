// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class ResetArm extends CommandBase {
  
  ArmSubsystem armSubsystem;

  public ResetArm(ArmSubsystem armSubsystem) {
    this.armSubsystem = armSubsystem;
    addRequirements(armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!armSubsystem.getCloseSwitch()){
      armSubsystem.moveCloseJoint(-0.2);
    }if(!armSubsystem.getFarSwitch()){
      armSubsystem.moveFarJoint(-0.2);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    armSubsystem.stopFarJoint();
    armSubsystem.stopCloseJoint();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return armSubsystem.getCloseSwitch() && armSubsystem.getFarSwitch();
  }
}
