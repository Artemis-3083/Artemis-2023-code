// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.photonvision.PhotonCamera;
import org.photonvision.RobotPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismObject2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.Balance;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.ElevatorDown;
import frc.robot.commands.ElevatorUp;
import frc.robot.commands.DriveForward;
import frc.robot.commands.DriveUntilDistanceFromTag;
import frc.robot.commands.SetElevatorHeight;
import frc.robot.commands.TurnToTag;
import frc.robot.commands.DriveForward;
import frc.robot.commands.DriveUntilDistanceFromTag;
import frc.robot.subsystems.DriveSystem;
import frc.robot.subsystems.ElevatorSystem;
import frc.robot.subsystems.LimelightSystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.VisionSystem;

public class Robot extends TimedRobot {
  
  ElevatorSystem elevator;
  DriveSystem driveSystem;
  LimelightSystem limelight;
  ArmSubsystem arm;
  PS4Controller driveController;
  PS4Controller controller;
  VisionSystem visionSystem;

  Mechanism2d mechanism2d = new Mechanism2d(3, 3);
  MechanismObject2d elevator2d;
  MechanismLigament2d firstArm;
  MechanismLigament2d secondArm;

  Command testCommand;

  @Override
  public void robotInit() {
    limelight = new LimelightSystem();
    driveSystem = new DriveSystem();
    arm = new ArmSubsystem();
    elevator = new ElevatorSystem();
    controller = new PS4Controller(0);
    visionSystem = new VisionSystem();

    //driveSystem.setDefaultCommand(new DriveCommand(driveSystem, controller));

    new POVButton(controller, 0).whileTrue(new ElevatorUp(elevator));
    new POVButton(controller, 180).whileTrue(new ElevatorDown(elevator));
    new JoystickButton(controller, PS4Controller.Button.kTriangle.value).toggleOnTrue(new TurnToTag(visionSystem, driveSystem));
    new JoystickButton(driveController, PS4Controller.Button.kCircle.value).toggleOnTrue(new Balance(driveSystem));

    testCommand = new SetElevatorHeight(0.8, elevator);
  
    MechanismRoot2d root = mechanism2d.getRoot("root", 2, 0);
    elevator2d = root.append(new MechanismLigament2d("Elevator", 1, 90, 6, new Color8Bit(Color.kOrange)));
    firstArm = elevator2d.append(new MechanismLigament2d("FirstArm", 0.5, 90, 6, new Color8Bit(Color.kCyan)));
    secondArm = firstArm.append(new MechanismLigament2d("SecondArm", 0.5, 90, 6, new Color8Bit(Color.kYellow)));

    SmartDashboard.putData("elevator", mechanism2d);

    //driveSystem.setDefaultCommand(new DriveUntilDistance(0.6, driveSystem, visionSystem));
  }
 
  @Override
  public void robotPeriodic() {

    SmartDashboard.putNumber("Pitch", driveSystem.getPitch());
    CommandScheduler.getInstance().run();
    //firstArm.setAngle(180 - arm.getCloseEncoderAngleDegrees());
    //secondArm.setAngle(180 - arm.getFarEncoderAngleDegrees());

    SmartDashboard.putString("TRANSFORM PHOTON", String.format("x: %.3f, y: %.3f, z: %.3f", visionSystem.getTagDistance(), visionSystem.getTagHight(), visionSystem.getTagAngle()));

  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {
    //new ArmUp(arm).schedule();
    //new DriveForward(driveSystem).withTimeout(1).andThen(new Balance(driveSystem)).schedule();
    if (testCommand != null) {
      testCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {

  }

  @Override
  public void autonomousExit() {
    if (testCommand != null) {
      testCommand.cancel();
    }
  }
  
  @Override
  public void teleopInit() {
    //new ElevatorDown(elevator).schedule();
    //new ArmDown(arm).schedule();
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}
}
