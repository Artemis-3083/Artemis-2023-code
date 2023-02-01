// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SubsysArm;

public class armdeggreas extends CommandBase {
  SubsysArm arm;

  PIDController pid = new PIDController(0, 0, 0);

  public armdeggreas(SubsysArm arm) {
    this.arm = new SubsysArm();
    addRequirements(arm);
  }

 
  @Override
  public void initialize() {
    arm.angle();
  }

  
  @Override
  public void execute() {
    pid.calculate(arm.degries(),arm.angle());
    arm.motorspeedincrees();
  }

  @Override
  public void end(boolean interrupted) {}

  
  @Override
  public boolean isFinished() {
    return pid.atSetpoint();
  }
}
