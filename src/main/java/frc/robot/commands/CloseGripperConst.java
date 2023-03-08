// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.GripperSystem;

public class CloseGripperConst extends CommandBase {
  
  GripperSystem gripperSystem;
  boolean almostThere;

  public CloseGripperConst(GripperSystem gripperSystem) {
    this.gripperSystem = gripperSystem;
    addRequirements(gripperSystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    almostThere = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putBoolean("almost there", almostThere);
    if(gripperSystem.getEncoder() < 0.5){
      almostThere = true;
    }else if(almostThere && gripperSystem.getEncoder() < 0.4){
      gripperSystem.stop();
    }/*else if(gripperSystem.getEncoder() < 0.5){
      gripperSystem.move(-0.5);
    }*/else{
      gripperSystem.move(-1);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    gripperSystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return gripperSystem.getEncoder() < 0.1;
    return false;
  }
}
