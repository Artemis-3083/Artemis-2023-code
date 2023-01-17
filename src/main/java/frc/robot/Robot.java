// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.Balance;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.DriveSystem;
import frc.robot.subsystems.LimelightSystem;

public class Robot extends TimedRobot {
  
  DriveSystem driveSystem;
  LimelightSystem limelight;
  PS4Controller controller;

  @Override
  public void robotInit() {
    limelight = new LimelightSystem();
    driveSystem = new DriveSystem();
    controller = new PS4Controller(0);

    driveSystem.setDefaultCommand(new DriveCommand(driveSystem, controller));

    new JoystickButton(controller, PS4Controller.Button.kCircle.value).whenPressed(new Balance());
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}
}
