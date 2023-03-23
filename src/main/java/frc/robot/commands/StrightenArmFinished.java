// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSystem;

public class StrightenArmFinished extends CommandBase {
  /** Creates a new StrightenArmFinished. */
  ArmSubsystem arm;
  ElevatorSystem elevator;
  public StrightenArmFinished(ArmSubsystem arm, ElevatorSystem elevator) {
    this.arm = arm;
    this.elevator = elevator;
    addRequirements(arm, elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Math.abs(arm.getCloseJoint()-125)<0.3 && Math.abs(arm.getFarJoint()-175)<0.3 && Math.abs(elevator.getHeight()+50)<0.3); // 0.3 tolerance
  }
}
