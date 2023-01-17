// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.SubsysArm;

public class armCommandUp extends CommandBase {
  SubsysArm armmovement;
  PS4Controller controller;

  public armCommandUp(SubsysArm armmovement) {
    this.armmovement = armmovement;
    controller = new PS4Controller(0);
    addRequirements(armmovement);
  }

  
  @Override
  public void initialize() {}

  
  @Override
  public void execute() {
    armmovement.motorspeeddecreas();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(armmovement.spied() != 0) {
      armmovement.motorstop();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
   if(!controller.getSquareButton()){
    return true;
   }

   return false;
  }
}
