// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CollectorSystem;
import frc.robot.subsystems.DriveSystem;
import frc.robot.subsystems.ElevatorSystem;
import frc.robot.subsystems.VisionSystem;

public class AutoScore extends CommandBase {
  
  CollectorSystem collectorSystem;
  DriveSystem driveSystem;
  ArmSubsystem armSubsystem;
  ElevatorSystem elevatorSystem;
  VisionSystem visionSystem;


  public AutoScore(CollectorSystem collectorSystem, ArmSubsystem armSubsystem, ElevatorSystem elevatorSystem, DriveSystem driveSystem) {
    this.armSubsystem = armSubsystem;
    this.collectorSystem = collectorSystem;
    this.elevatorSystem = elevatorSystem;
    this.driveSystem = driveSystem;
    addRequirements(elevatorSystem, armSubsystem, collectorSystem, driveSystem);
  }

  @Override
  public void initialize() {
    new DriveUntilDistanceFromTag(1, driveSystem, visionSystem).schedule();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {} //open to 1.58 m

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
