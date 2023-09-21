// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismObject2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
// import frc.robot.commands.AlwaysPID;
import frc.robot.commands.ArmPID;
import frc.robot.commands.ArmPIDForAuto;
import frc.robot.commands.Balance;
import frc.robot.commands.Blow;
import frc.robot.commands.CloseCloseJoint;
import frc.robot.commands.CloseFarJoint;
import frc.robot.commands.CloseGripper;
import frc.robot.commands.CloseGripperAmp;
import frc.robot.commands.CloseGripperConst;
import frc.robot.commands.Collect;
import frc.robot.commands.DriveBackwardScore;
import frc.robot.commands.DriveBackwards;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.DriveForwardBalance;
import frc.robot.commands.ElevatorDown;
import frc.robot.commands.ElevatorPID;
import frc.robot.commands.ElevatorUp;
import frc.robot.commands.GripperPID;
import frc.robot.commands.OpenCloseJoint;
import frc.robot.commands.OpenFarJoint;
import frc.robot.commands.OpenGripper;
import frc.robot.commands.ResetArm;
// import frc.robot.commands.ResetArmAndElevator;
import frc.robot.commands.ResetDriveEncoders;
import frc.robot.commands.ResetGripperEncoders;
import frc.robot.commands.SetGripperEncoderTo1;
import frc.robot.commands.ShootCube;
import frc.robot.commands.ResetElevator;
import frc.robot.commands.DriveForwardCommunity;
// import frc.robot.commands.DriveUntilDistanceFromTag;
import frc.robot.commands.Suck;
// import frc.robot.commands.DriveUntilDistanceFromTag;
import frc.robot.subsystems.DriveSystem;
import frc.robot.subsystems.ElevatorSystem;
import frc.robot.subsystems.LimelightSystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GripperSystem;
// import frc.robot.subsystems.VisionSystem;

public class Robot extends TimedRobot {
  
  GripperSystem gripperSystem;
  ElevatorSystem elevatorSystem;
  DriveSystem driveSystem;
  LimelightSystem limelight;
  ArmSubsystem armSystem;
  PS4Controller driveController;
  PS4Controller controller;
  Command resetCommand;
  Command strightenArm;
  Command lowerArm;
  Command cubeMode;
  Command coneMode;
  Command shootCube;
  Command armBalanceMode;

  Command tryOutPID;
  double farGoal = 0, closeGoal = 0, elevatorGoal = 0;

  SendableChooser<String> autoChooser;
  SendableChooser<String> armChooser;

  UsbCamera camera;
  CvSource stream;

  @Override
  public void robotInit() {

    // Systems:
    gripperSystem = new GripperSystem();
    driveSystem = new DriveSystem();
    armSystem = new ArmSubsystem();
    elevatorSystem = new ElevatorSystem();
    controller = new PS4Controller(2);
    driveController = new PS4Controller(0);

    // Vision:
    limelight = new LimelightSystem();
    camera = CameraServer.startAutomaticCapture();
    camera.setResolution(426, 240);
    

    shootCube = new ShootCube(gripperSystem);

    // Arm states:
    resetCommand = (new ResetArm(armSystem).andThen(new ArmPID(2, 2, armSystem))).alongWith((new ResetElevator(elevatorSystem)).andThen(new ElevatorPID(-10, elevatorSystem)));
    strightenArm = new ArmPIDForAuto(125, 168, armSystem).alongWith(new ElevatorPID(-50, elevatorSystem));
    armBalanceMode = new ArmPID(40, 2, armSystem).alongWith(new ElevatorPID(-245.42, elevatorSystem));
    lowerArm = new ArmPID(11.038, 75.563, armSystem).alongWith(new ElevatorPID(-204, elevatorSystem));



    //auto chooser in Shuffleboard:
    autoChooser = new SendableChooser<>();
    autoChooser.setDefaultOption("Default - Nothing", "resetEncoders");
    autoChooser.addOption("Balance", "balance");
    autoChooser.addOption("Community + balance", "communityAndBalance");
    autoChooser.addOption("Score + balance", "scoreAndBalance");
    autoChooser.addOption("Score", "score");
    autoChooser.addOption("Score + community", "scoreAndCommunity");
    autoChooser.addOption("Score + community + balance", "scoreAndCommunityAndBalance");
    SmartDashboard.putData("Auto choosing", autoChooser);

    armChooser = new SendableChooser<>();
    armChooser.setDefaultOption("Default - with arm", "withArm");
    armChooser.addOption("Without arm", "withoutArm");
    SmartDashboard.putData("Arm choosing", armChooser);


    //Buttons:
    //Arm:
    new JoystickButton(controller, PS4Controller.Button.kCross.value).toggleOnTrue(lowerArm);
    new JoystickButton(controller, PS4Controller.Button.kTriangle.value).toggleOnTrue(strightenArm);
    new JoystickButton(controller, PS4Controller.Button.kSquare.value).toggleOnTrue(armBalanceMode);
    new JoystickButton(controller, PS4Controller.Button.kCircle.value).toggleOnTrue(resetCommand);
    
    //Gripper:
    new POVButton(controller, 0).toggleOnTrue(new GripperPID(0.3, gripperSystem));
    new POVButton(controller, 180).toggleOnTrue(new GripperPID(0, gripperSystem));

    new POVButton(controller, 90).toggleOnTrue(new OpenGripper(gripperSystem));
    new POVButton(controller, 270).toggleOnTrue(new CloseGripperAmp(gripperSystem));
    
    new JoystickButton(controller, PS4Controller.Button.kR1.value).whileTrue(new Collect(gripperSystem));
    new JoystickButton(controller, PS4Controller.Button.kL1.value).whileTrue(new Blow(gripperSystem));

    //Balance:
    new JoystickButton(driveController, PS4Controller.Button.kSquare.value).whileTrue(new Balance(driveSystem));

    //Drive:
    driveSystem.setDefaultCommand(new DriveCommand(driveSystem, driveController));
  }
 
  @Override
  public void robotPeriodic() {

    // Input for Shuffleboard:
    SmartDashboard.putNumber("Close joint encoder", armSystem.getCloseJoint());
    SmartDashboard.putNumber("Far joint encoder", armSystem.getFarJoint());
    SmartDashboard.putBoolean("Limit switch close joint", armSystem.getCloseSwitch());
    SmartDashboard.putBoolean("Limit switch far joint", armSystem.getFarSwitch());
    SmartDashboard.putNumber("Elevator hight", elevatorSystem.getHeight());
    SmartDashboard.putNumber("Pitch", driveSystem.getPitch());
    SmartDashboard.putNumber("Right front drive encoder", driveSystem.getDistancePassedRightM());
    
    CommandScheduler.getInstance().run();
  }
  
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {

    /*
     * scoreState1 - holding the cube and getting the arm to the right position to score 
     * scoreState2 - shooting the cube, then driving backwards while reseting the arm
     * scoreState3 - balancing on the charge station while getting the arm into a certian position?
     * communityState - driving backwards while putting the arm to balance state
    */

    Command scoreState1 = new GripperPID(0.3, gripperSystem).alongWith(new ArmPIDForAuto(125, 162, armSystem).alongWith(new ElevatorPID(-20, elevatorSystem)));
    Command scoreState2 = new ShootCube(gripperSystem).withTimeout(0.5).andThen(new DriveBackwardScore(driveSystem)).withTimeout(2).alongWith(resetCommand);
    Command scoreState3 = new Balance(driveSystem).alongWith(new ArmPID(40, 2, armSystem).alongWith(new ElevatorPID(-245.42, elevatorSystem)));

    Command communityState = new DriveBackwardScore(driveSystem).withTimeout(2).alongWith(armBalanceMode);

    //getting the auto from Shuffleboard:
    String armSelected = armChooser.getSelected();
    String autoSelected = autoChooser.getSelected();

    //
    Command autoCommand = new ResetDriveEncoders(driveSystem);

    if(armSelected.equals("withArm")){
      if(autoSelected.equals("balance")){
        // autoCommand = balanceState1;
        // autoCommand = autoCommand.andThen(autoBalanceState.withTimeout(3)).andThen(armBalanceMode.alongWith(new Balance(driveSystem)));
        // autoCommand = autoCommand.andThen((resetCo048mmand.withTimeout(2).andThen(armBalanceMode)).alongWith(new DriveForwardBalance(driveSystem).andThen(new Balance(driveSystem))));
      }else if(autoSelected.equals("communityAndBalance")){
        // autoCommand = autoResetCommand;
        // autoCommand = (.andThen(new DriveBackwards(driveSystem))).andThen(new DriveBackwards(driveSystem)).andThen(new Balance(driveSystem));
        // autoCommand = autoCommand.andThen((autoResetCommand.withTimeout(2).andThen(armBalanceMode)).alongWith(new DriveForwardCommunity(driveSystem).andThen(new DriveBackwardScore(driveSystem)).andThen(new Balance(driveSystem))));
      }else if(autoSelected.equals("scoreAndBalance")){
        // autoCommand = scoreState3;
        autoCommand = scoreState1.withTimeout(3).andThen(scoreState2).withTimeout(7).andThen(scoreState3);
      }else if(autoSelected.equals("score")){
        autoCommand = scoreState1.withTimeout(5).andThen(scoreState2);
        // autoCommand = strightenArm.alongWith(new GripperPID(0.3, gripperSystem));//autoCommand.andThen(gripperState).alongWith(strightenArm);//.andThen(new ShootCube(gripperSystem).withTimeout(0.5)).withTimeout(3).andThen(resetCommand);
      }else if(autoSelected.equals("scoreAndCommunity")){//dis community
        autoCommand = new DriveForwardCommunity(driveSystem).andThen(communityState).withTimeout(5.5).andThen(new Balance(driveSystem));
        // autoCommand = scoreState1.withTimeout(5).andThen(scoreState2).andThen(new DriveBackwardScore(driveSystem));
      }else if(autoSelected.equals("scoreAndCommunityAndBalance")){
  
      }
    }else if(armSelected.equals("withoutArm")){
      if(autoSelected.equals("balance")){
        autoCommand = autoCommand.andThen(new DriveForwardBalance(driveSystem)).andThen(new Balance(driveSystem));
      }else if(autoSelected.equals("communityAndBalance")){
        autoCommand = autoCommand.andThen(new DriveForwardCommunity(driveSystem)).andThen().andThen(new Balance(driveSystem));
      }
    }
    //Command farJoint = new ResetFarJoint(armSystem).andThen(new FarJointPID(5, armSystem));
    //Command closeJoint = new ResetCloseJoint(armSystem).andThen(new CloseJointPID(10, armSystem));

    // Command elevator = new ResetElevator(elevatorSystem).andThen(new ElevatorPID(-10, elevatorSystem));
    // Command allPID = new LowerArm(armSystem).andThen(new ArmPID(90, 120, armSystem)).alongWith(elevator);
    
    // new SetGripperEncoderTo1(gripperSystem).schedule();
    autoCommand.schedule();

    //autonomus.schedule();

    // new DriveForward(driveSystem).schedule();
    //actual auto command
    // resetCommand.withTimeout(3.5).andThen(new DriveForward(driveSystem)).andThen(new DriveBackwards(driveSystem)).andThen(new Balance(driveSystem)).schedule();
    // new ResetDriveEncoders(driveSystem).schedule();


    //new ResetDriveEncoders(driveSystem).andThen(new DriveForward(driveSystem)).andThen(new DriveBackwards(driveSystem)).andThen(new Balance(driveSystem)).schedule();
    
    
    // resetCommand.schedule();
      //elevator.schedule();
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
