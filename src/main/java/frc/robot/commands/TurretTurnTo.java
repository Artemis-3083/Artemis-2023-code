// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.TurretSystem;

public class TurretTurnTo extends CommandBase {
  TurretSystem turretSystem;
  private double destination, speed;
  

  public TurretTurnTo(TurretSystem turretSystem, double destination) {
    this.destination = destination;
    this.turretSystem = turretSystem;
    addRequirements(turretSystem);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    speed = Math.min(Math.abs(turretSystem.getAngle() - destination)/90, 1);
    if (turretSystem.getAngle() < destination){
      turretSystem.rotate(speed);
    } else {
      turretSystem.rotate(-speed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    turretSystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return turretSystem.getAngle() >= destination-5 && turretSystem.getAngle() <= destination+5 ||
    turretSystem.getAngle() >= Constants.TurretTurnLimit-5 ||
    turretSystem.getAngle() <= Constants.TurretTurnLimit+5;
  }
}
