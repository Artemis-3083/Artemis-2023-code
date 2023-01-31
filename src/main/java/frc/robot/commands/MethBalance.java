// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSystem;
import frc.robot.subsystems.PLEASE_GOD_HELPME_END_MY_SUFFERING;

public class MethBalance extends CommandBase {

  PLEASE_GOD_HELPME_END_MY_SUFFERING methSystem;
  DriveSystem driveSystem;
  
  public MethBalance(PLEASE_GOD_HELPME_END_MY_SUFFERING methSystem, DriveSystem driveSystem) {
    this.driveSystem = driveSystem;
    this.methSystem = methSystem;
    addRequirements(methSystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(methSystem.getPitch() > 0){
      driveSystem.drive(methSystem.yes(), 0, 0);
    }else if(methSystem.getPitch() < 0){
      driveSystem.drive(0, methSystem.yes(), 0);
    } 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return methSystem.yes() > 11 && methSystem.yes() < -11;
  }
}
