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
import frc.robot.commands.CloseArm;
import frc.robot.commands.CloseCloseJoint;
import frc.robot.commands.CloseFarJoint;
//import frc.robot.commands.CollectGamePiece;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.ElevatorDown;
import frc.robot.commands.ElevatorUp;
import frc.robot.commands.OpenArm;
import frc.robot.commands.OpenCloseJoint;
import frc.robot.commands.OpenFarJoint;
//import frc.robot.commands.ReleaseGamePiece;
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
//import frc.robot.subsystems.CollectorSystem;
import frc.robot.subsystems.VisionSystem;

public class Robot extends TimedRobot {
  
  //CollectorSystem collectorSystem;
  ElevatorSystem elevatorSystem;
  DriveSystem driveSystem;
  LimelightSystem limelight;
  ArmSubsystem armSystem;
  PS4Controller driveController;
  PS4Controller controller;
  VisionSystem visionSystem;

  /*Mechanism2d mechanism2d = new Mechanism2d(3, 3);
  MechanismObject2d elevator2d;
  MechanismLigament2d firstArm;
  MechanismLigament2d secondArm;

  Command testCommand;*/

  @Override
  public void robotInit() {
    //collectorSystem = new CollectorSystem();
    limelight = new LimelightSystem();
    driveSystem = new DriveSystem();
    armSystem = new ArmSubsystem();
    elevatorSystem = new ElevatorSystem();
    controller = new PS4Controller(2);
    visionSystem = new VisionSystem();
    driveController = new PS4Controller(0);

    driveSystem.setDefaultCommand(new DriveCommand(driveSystem, driveController));

    //elevator.setDefaultCommand(new SetElevatorHeight(20, elevator));
    new POVButton(controller, 0).whileTrue(new ElevatorUp(elevatorSystem));
    new POVButton(controller, 180).whileTrue(new ElevatorDown(elevatorSystem));
    /*new JoystickButton(controller, PS4Controller.Button.kTriangle.value).toggleOnTrue(new TurnToTag(visionSystem, driveSystem));
    new JoystickButton(controller, PS4Controller.Button.kCross.value).onTrue(new DriveUntilDistanceFromTag(1, driveSystem, visionSystem));
    new JoystickButton(driveController, PS4Controller.Button.kCircle.value).toggleOnTrue(new Balance(driveSystem));*/
    new POVButton(controller, 90).whileTrue(new OpenCloseJoint(armSystem));
    new POVButton(controller, 270).whileTrue(new CloseCloseJoint(armSystem));
    /*new JoystickButton(controller, PS4Controller.Axis.kR2.value).whileTrue(new CollectGamePiece(collectorSystem));
    new JoystickButton(controller, PS4Controller.Axis.kL2.value).whileTrue(new ReleaseGamePiece(collectorSystem));*/

    /*testCommand = new SetElevatorHeight(0.8, elevator);
  
    MechanismRoot2d root = mechanism2d.getRoot("root", 2, 0);
    elevator2d = root.append(new MechanismLigament2d("Elevator", 1, 90, 6, new Color8Bit(Color.kOrange)));
    firstArm = elevator2d.append(new MechanismLigament2d("FirstArm", 0.5, 90, 6, new Color8Bit(Color.kCyan)));
    secondArm = firstArm.append(new MechanismLigament2d("SecondArm", 0.5, 90, 6, new Color8Bit(Color.kYellow)));

    SmartDashboard.putData("elevator", mechanism2d);*/

    //driveSystem.setDefaultCommand(new DriveUntilDistanceFromTag(0.6, driveSystem, visionSystem));
  }
 
  @Override
  public void robotPeriodic() {

    SmartDashboard.putNumber("close distance", armSystem.getCloseJoint());
    SmartDashboard.putNumber("far distance", armSystem.getFarJoint());
    SmartDashboard.putBoolean("close limit", armSystem.getCloseSwitch());
    SmartDashboard.putBoolean("far limit", armSystem.getFarSwitch());
    SmartDashboard.putNumber("elevator hight", elevatorSystem.getHeight());
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
    //new DriveForward(driveSystem).withTimeout(1).andThen(new Balance(driveSystem)).schedule();
    /*if (autoCommand != null) {
      autoCommand.schedule();
    }*/
  }

  @Override
  public void autonomousPeriodic() {

  }

  @Override
  public void autonomousExit() {
    /*if (autoCommand != null) {
      autoCommand.cancel();
    }*/
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
