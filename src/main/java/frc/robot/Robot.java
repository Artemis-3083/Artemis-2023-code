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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.Balance;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.DriveForward;
import frc.robot.commands.MethBalance;
import frc.robot.subsystems.DriveSystem;
import frc.robot.subsystems.LimelightSystem;
import frc.robot.subsystems.PLEASE_GOD_HELPME_END_MY_SUFFERING;

public class Robot extends TimedRobot {
  
  PLEASE_GOD_HELPME_END_MY_SUFFERING altDriveSystem;
  DriveSystem driveSystem;
  LimelightSystem limelight;
  PS4Controller controller;

  PhotonCamera camera;

  @Override
  public void robotInit() {
    limelight = new LimelightSystem();
    driveSystem = new DriveSystem();
    controller = new PS4Controller(0);
    altDriveSystem = new PLEASE_GOD_HELPME_END_MY_SUFFERING();

   
    driveSystem.setDefaultCommand(new DriveCommand(driveSystem, controller));
    altDriveSystem.setDefaultCommand(new MethBalance(altDriveSystem, driveSystem));

    camera = new PhotonCamera("Microsoft_LifeCam_Studio(TM)");
    camera.setDriverMode(false);
    camera.setPipelineIndex(0);
    
  }
 
  @Override
  public void robotPeriodic() {

    SmartDashboard.putNumber("Pitch", driveSystem.getPitch());
    CommandScheduler.getInstance().run();

    PhotonPipelineResult result = camera.getLatestResult();
    //var result2 = camera.getLatestResult();
    if (result.hasTargets()) {
      PhotonTrackedTarget trackedResult = result.getBestTarget();
      Transform3d transform3d = trackedResult.getBestCameraToTarget();
      SmartDashboard.putString("TRANSFORM PHOTON", String.format("x: %.3f, y: %.3f, z: %.3f", transform3d.getX(), transform3d.getY(), transform3d.getZ()));
    }
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {
    new DriveForward(driveSystem).withTimeout(1).andThen(new Balance(driveSystem)).schedule();;
  }

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
