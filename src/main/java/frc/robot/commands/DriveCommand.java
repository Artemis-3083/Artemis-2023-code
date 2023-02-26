// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSystem;

public class DriveCommand extends CommandBase {

  DriveSystem driveSystem;
  PS4Controller controller;
  
  public DriveCommand(DriveSystem driveSystem, PS4Controller controller) {
    this.driveSystem = driveSystem;
    this.controller = controller;
    addRequirements(driveSystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    //driveSystem.tankDrive(controller.getRightX(), controller.getLeftX());
    driveSystem.drive(controller.getR2Axis(), controller.getL2Axis(), controller.getRightX());
  }

  @Override
  public void end(boolean interrupted) {
    driveSystem.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
