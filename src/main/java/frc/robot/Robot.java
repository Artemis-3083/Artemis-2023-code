// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Joystick;
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
import frc.robot.commands.AllPID;
import frc.robot.commands.ArmPID;
import frc.robot.commands.Balance;
import frc.robot.commands.BalanceCommand;
import frc.robot.commands.CloseArm;
import frc.robot.commands.CloseCloseJoint;
import frc.robot.commands.CloseFarJoint;
import frc.robot.commands.CloseGripper;
import frc.robot.commands.CloseJointPID;
import frc.robot.commands.DriveBackwards;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.ElevatorDown;
import frc.robot.commands.ElevatorPID;
import frc.robot.commands.ElevatorUp;
import frc.robot.commands.FarJointPID;
import frc.robot.commands.GripperPID;
import frc.robot.commands.OpenArm;
import frc.robot.commands.OpenCloseJoint;
import frc.robot.commands.OpenFarJoint;
import frc.robot.commands.OpenGripper;
import frc.robot.commands.ResetArm;
import frc.robot.commands.ResetCloseJoint;
import frc.robot.commands.ResetDriveEncoders;
import frc.robot.commands.ResetFarJoint;
import frc.robot.commands.ResetGripper;
import frc.robot.commands.ResetElevator;
import frc.robot.commands.DriveForward;
import frc.robot.commands.DriveUntilDistanceFromTag;
import frc.robot.commands.SetElevatorHeight;
import frc.robot.commands.SetFarJointAngle;
import frc.robot.commands.Suck;
import frc.robot.commands.TurnToTag;
import frc.robot.commands.DriveForward;
import frc.robot.commands.DriveUntilDistanceFromTag;
import frc.robot.subsystems.DriveSystem;
import frc.robot.subsystems.ElevatorSystem;
import frc.robot.subsystems.LimelightSystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GripperSystem;
import frc.robot.subsystems.VisionSystem;

public class Robot extends TimedRobot {
  
  GripperSystem gripperSystem;
  ElevatorSystem elevatorSystem;
  DriveSystem driveSystem;
  LimelightSystem limelight;
  ArmSubsystem armSystem;
  PS4Controller driveController;
  PS4Controller controller;
  VisionSystem visionSystem;
  Command resetCommand;
  Command strightenArm;
  Command lowerArm;

  /*Mechanism2d mechanism2d = new Mechanism2d(3, 3);
  MechanismObject2d elevator2d;
  MechanismLigament2d firstArm;
  MechanismLigament2d secondArm;

  Command testCommand;*/

  @Override
  public void robotInit() {
    gripperSystem = new GripperSystem();
    limelight = new LimelightSystem();
    driveSystem = new DriveSystem();
    armSystem = new ArmSubsystem();
    elevatorSystem = new ElevatorSystem();
    controller = new PS4Controller(2);
    visionSystem = new VisionSystem();
    driveController = new PS4Controller(0);

    resetCommand = new ResetArm(armSystem).alongWith(new ResetElevator(elevatorSystem));
    strightenArm = new ArmPID(90, 120, armSystem).alongWith(new ElevatorPID(-10, elevatorSystem));
    lowerArm = new ArmPID(9.892, 70.175, armSystem).alongWith(new ElevatorPID(-178.895, elevatorSystem));
//
//close 49684518701
//far 

//close 123.91297763868558
  //146.87359831778787
  //
    /*strightenArm = new AllPID(armSystem, elevatorSystem, 140, 146.873, -34.674);
    resetCommand = new ResetArm(armSystem).alongWith(new ResetElevator(elevatorSystem)).andThen(new AllPID(armSystem, elevatorSystem, 3, 3, -2));
    lowerArm = new AllPID(armSystem, elevatorSystem, 18.659, 71.418, -150.06);


    new JoystickButton(controller, PS4Controller.Button.kCross.value).toggleOnTrue(lowerArm);
    new JoystickButton(controller, PS4Controller.Button.kTriangle.value).toggleOnTrue(strightenArm);
    new JoystickButton(controller, PS4Controller.Button.kCircle.value).toggleOnTrue(resetCommand);*/


    //new JoystickButton(controller, PS4Controller.Button.kSquare.value).toggleOnTrue();
    driveSystem.setDefaultCommand(new DriveCommand(driveSystem, driveController));
    //elevatorSystem.setDefaultCommand(new SetElevatorHeight(20, elevatorSystem));
    // new POVButton(controller, 0).whileTrue(new ElevatorDown(elevatorSystem));
    // new POVButton(controller, 180).whileTrue(new ElevatorUp(elevatorSystem));


    /*new POVButton(controller, 90).whileTrue(new OpenGripper(gripperSystem));
    new POVButton(controller, 270).whileTrue(new CloseGripper(gripperSystem));

    new POVButton(controller, 0).whileTrue(new ElevatorUp(elevatorSystem));
    new POVButton(controller, 180).whileTrue(new ElevatorDown(elevatorSystem));

    new JoystickButton(controller, PS4Controller.Button.kR2.value).whileTrue(new OpenCloseJoint(armSystem));
    new JoystickButton(controller, PS4Controller.Button.kL2.value).whileTrue(new CloseCloseJoint(armSystem));

    new JoystickButton(controller, PS4Controller.Button.kR1.value).whileTrue(new OpenFarJoint(armSystem));
    new JoystickButton(controller, PS4Controller.Button.kL1.value).whileTrue(new CloseFarJoint(armSystem));*/

    new JoystickButton(driveController,PS4Controller.Button.kCircle.value).whileTrue(new Balance(driveSystem));
    new JoystickButton(controller, PS4Controller.Button.kSquare.value).whileTrue(new Suck(gripperSystem));
    //new JoystickButton(controller, PS4Controller.Button.kR2.value).whileTrue(new ResetGripper(gripperSystem));
    /*new JoystickButton(controller, PS4Controller.Button.kTriangle.value).toggleOnTrue(new TurnToTag(visionSystem, driveSystem));
    new JoystickButton(controller, PS4Controller.Button.kCross.value).onTrue(new DriveUntilDistanceFromTag(1, driveSystem, visionSystem));
    new JoystickButton(driveController, PS4Controller.Button.kCircle.value).toggleOnTrue(new Balance(driveSystem));*/
    // new POVButton(controller, 90).whileTrue(new OpenFarJoint(armSystem));
    // new POVButton(controller, 270).whileTrue(new CloseFarJoint(armSystem));
    /*new JoystickButton(controller, PS4Controller.Axis.kR2.value).whileTrue(new CollectGamePiece(collectorSystem));
    new JoystickButton(controller, PS4Controller.Axis.kL2.value).whileTrue(new ReleaseGamePiece(collectorSystem));*/

    // new JoystickButton(controller, PS4Controller.Button.kCircle.value).whileTrue(new ResetFarJoint(armSystem));
    /*testCommand = new SetElevatorHeight(0.8, elevator);
  
    MechanismRoot2d root = mechanism2d.getRoot("root", 2, 0);
    elevator2d = root.append(new MechanismLigament2d("Elevator", 1, 90, 6, new Color8Bit(Color.kOrange)));
    firstArm = elevator2d.append(new MechanismLigament2d("FirstArm", 0.5, 90, 6, new Color8Bit(Color.kCyan)));
    secondArm = firstArm.append(new MechanismLigament2d("SecondArm", 0.5, 90, 6, new Color8Bit(Color.kYellow)));

    SmartDashboard.putData("elevator", mechanism2d);*/

    //driveSystem.setDefaultCommand(new DriveUntilDistanceFromTag(0.6, driveSystem, visionSystem));
    // resetCommand.schedule();
  }
 
  @Override
  public void robotPeriodic() {
    new Suck(gripperSystem).schedule();
    SmartDashboard.putNumber("close distance", armSystem.getCloseJoint());
    SmartDashboard.putNumber("far distance", armSystem.getFarJoint());
    SmartDashboard.putBoolean("close limit", armSystem.getCloseSwitch());
    SmartDashboard.putBoolean("far limit", armSystem.getFarSwitch());
    SmartDashboard.putNumber("elevator hight", elevatorSystem.getHeight());
    SmartDashboard.putNumber("Pitch", driveSystem.getPitch());
    SmartDashboard.putNumber("drive encoders", driveSystem.getDistancePassedRightM());
    CommandScheduler.getInstance().run();
    //firstArm.setAngle(180 - arm.getCloseEncoderAngleDegrees());
    //secondArm.setAngle(180 - arm.getFarEncoderAngleDegrees());

    SmartDashboard.putString("TRANSFORM PHOTON", String.format("x: %.3f, y: %.3f, z: %.3f", visionSystem.getTagDistance(), visionSystem.getTagHight(), visionSystem.getTagAngle()));

  }
  /*
   * robotinit - eipus 332
   * button - hail
   * button - floor
   * button - eipus 332
   */

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {
    //Command farJoint = new ResetFarJoint(armSystem).andThen(new FarJointPID(5, armSystem));
    //Command closeJoint = new ResetCloseJoint(armSystem).andThen(new CloseJointPID(10, armSystem));


    Command elevator = new ResetElevator(elevatorSystem).andThen(new ElevatorPID(-10, elevatorSystem));
    Command allPID = new ResetArm(armSystem).andThen(new ArmPID(90, 120, armSystem)).alongWith(elevator);
    
    // new DriveForward(driveSystem).schedule();

    //actual auto command
    // resetCommand.withTimeout(3.5).andThen(new DriveForward(driveSystem)).andThen(new DriveBackwards(driveSystem)).andThen(new Balance(driveSystem)).schedule();
    // new ResetDriveEncoders(driveSystem).schedule();
    new ResetDriveEncoders(driveSystem).andThen(new DriveForward(driveSystem)).andThen(new DriveBackwards(driveSystem)).andThen(new Balance(driveSystem)).schedule();
    
    // new BalanceCommand(driveSystem).schedule();
    // resetCommand.schedule();
    // new DriveForward(driveSystem).schedule();
    // new Balance(driveSystem).schedule();
    //closeJoint.andThen(farjoint()).alongWith(elevator).schedule();
    //new ResetElevator(elevatorSystem).andThen(new ElevatorPID(-10, elevatorSystem)).schedule();
    //arm.alongWith(elevator).schedule();
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
