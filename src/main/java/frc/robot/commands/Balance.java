// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSystem;

public class Balance extends CommandBase {
  
  DriveSystem driveSystem;

  public Balance() {
    driveSystem = new DriveSystem();
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if(driveSystem.getPitch() > 0){
      driveSystem.drive(0.1, 0, 0);
    }else{
      driveSystem.drive(0, 0.1, 0);
    }
    System.out.println("called check");
  }

  @Override
  public void end(boolean interrupted) {
    driveSystem.stop();
  }

  @Override
  public boolean isFinished() {
    return 2 > driveSystem.getPitch() && driveSystem.getPitch() > -2;
  }
}
