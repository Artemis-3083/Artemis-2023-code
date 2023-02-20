// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SubsysArm;

public class aimarm extends CommandBase {
SubsysArm arm;
double angle;

  public aimarm(SubsysArm arm) {
    this.arm = arm;
  }
  
  @Override
  public void initialize() {
    angle =  Math.atan(arm.length()/ arm.hight());
  }

  
  @Override
  public void execute() {
    if(arm.farEncoderget() < 90) {
      arm.farSpark(0.25);
    }
    else {
      arm.farSpark(-0.25);
    }
    
    if(arm.closeEncoder() > angle) {
      arm.closeSpark(-0.25);
    }
    else {
      arm.closeSpark(0.25);
    }
    
  }
  
  @Override
  public void end(boolean interrupted) {
    arm.moveBoth(0);
  }

 
  @Override
  public boolean isFinished() {
    return arm.farEncoderget() == 90 && arm.closeEncoder() == angle;
  }
}
