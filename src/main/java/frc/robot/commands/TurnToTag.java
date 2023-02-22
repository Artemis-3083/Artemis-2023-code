// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSystem;
import frc.robot.subsystems.VisionSystem;

public class TurnToTag extends CommandBase {
  
  VisionSystem visionSystem;
  DriveSystem driveSystem;
  
  public TurnToTag(VisionSystem visionSystem, DriveSystem driveSystem) {
    this.visionSystem = visionSystem;
    this.driveSystem = driveSystem;
    addRequirements(visionSystem, driveSystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if(visionSystem.getTagAngle() > 0){
      driveSystem.drive(0, 0, 0.5);
    }else{
      driveSystem.drive(0, 0, -0.5);
    }
  }

  @Override
  public void end(boolean interrupted) {
    driveSystem.stop();
  }

  @Override
  public boolean isFinished() {
    return visionSystem.getTagAngle() < 5 && visionSystem.getTagAngle() > -5;
  }
}
