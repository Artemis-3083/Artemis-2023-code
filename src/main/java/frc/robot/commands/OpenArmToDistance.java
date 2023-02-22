// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class OpenArmToDistance extends CommandBase {
  
  ArmSubsystem armSubsystem;
  double distance;
  double place;

  public OpenArmToDistance(ArmSubsystem armSubsystem, double distance) {
    this.armSubsystem = armSubsystem;
    this.distance = distance;
    addRequirements(armSubsystem);
  }

  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    place = armSubsystem.getCloseJoint() + armSubsystem.getFarJoint();
    if(place < distance){
      armSubsystem.moveCloseJoint(0.3);
      armSubsystem.moveFarJoint(0.3);
    }else{
      armSubsystem.moveCloseJoint(-0.3);
      armSubsystem.moveFarJoint(-0.3);
    }
  }

  @Override
  public void end(boolean interrupted) {
    armSubsystem.stopCloseJoint();
    armSubsystem.stopFarJoint();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return place > distance-5 && place < distance+5;
  }
}
