// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.GripperSystem;

public class CloseGripperAmp extends CommandBase {
  
  GripperSystem gripperSystem;

  public CloseGripperAmp(GripperSystem gripperSystem) {
    this.gripperSystem = gripperSystem;
    addRequirements(gripperSystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(gripperSystem.getCurrent() >= 16){
      gripperSystem.stop();
    }else{
      gripperSystem.move(-1);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    gripperSystem.resetEncoder();
    gripperSystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return gripperSystem.getCurrent() >= 16;
  }
}
