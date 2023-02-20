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
import frc.robot.commands.ArmDown;
import frc.robot.commands.ArmUp;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.Balance;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.DriveForward;
import frc.robot.commands.DriveUntilDistance;
import frc.robot.subsystems.DriveSystem;
import frc.robot.subsystems.LimelightSystem;
import frc.robot.subsystems.SubsysArm;
import frc.robot.subsystems.VisionSystem;

public class Robot extends TimedRobot {
  
  DriveSystem driveSystem;
  LimelightSystem limelight;
  SubsysArm arm;
  PS4Controller controller;
  VisionSystem visionSystem;

  Mechanism2d mechanism2d = new Mechanism2d(3, 3);
  MechanismObject2d elevator;
  MechanismLigament2d firstArm;
  MechanismLigament2d secondArm;

  @Override
  public void robotInit() {
    limelight = new LimelightSystem();
    driveSystem = new DriveSystem();
    arm = new SubsysArm();
    controller = new PS4Controller(0);
    visionSystem = new VisionSystem();

    driveSystem.setDefaultCommand(new DriveCommand(driveSystem, controller));

    /*MechanismRoot2d root = mechanism2d.getRoot("root", 2, 0);
    elevator = root.append(new MechanismLigament2d("Elevator", 1, 90, 6, new Color8Bit(Color.kOrange)));
    firstArm = elevator.append(new MechanismLigament2d("FirstArm", 0.5, 90, 6, new Color8Bit(Color.kCyan)));
    secondArm = firstArm.append(new MechanismLigament2d("SecondArm", 0.5, 90, 6, new Color8Bit(Color.kYellow)));

    SmartDashboard.putData("elevator", mechanism2d);*/
    driveSystem.setDefaultCommand(new DriveUntilDistance(0.6, driveSystem, visionSystem));
  }
 
  @Override
  public void robotPeriodic() {

    SmartDashboard.putNumber("Pitch", driveSystem.getPitch());
    CommandScheduler.getInstance().run();
    //firstArm.setAngle(180 - arm.getCloseEncoderAngleDegrees());
    //secondArm.setAngle(180 - arm.getFarEncoderAngleDegrees());

    SmartDashboard.putString("TRANSFORM PHOTON", String.format("x: %.3f, y: %.3f, z: %.3f", visionSystem.getDistance(), visionSystem.getHight(), visionSystem.getAngle()));

  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {
    //new ArmUp(arm).schedule();
    new DriveForward(driveSystem).withTimeout(1).andThen(new Balance(driveSystem)).schedule();
  }

  @Override
  public void autonomousPeriodic() {

  }

  @Override
  public void teleopInit() {
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
