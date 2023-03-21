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
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.AlwaysPID;
import frc.robot.commands.ArmPID;
import frc.robot.commands.Balance;
import frc.robot.commands.Blow;
import frc.robot.commands.CloseCloseJoint;
import frc.robot.commands.CloseFarJoint;
import frc.robot.commands.CloseGripper;
import frc.robot.commands.CloseGripperAmp;
import frc.robot.commands.CloseGripperConst;
import frc.robot.commands.Collect;
import frc.robot.commands.DriveBackwardScore;
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
import frc.robot.commands.ResetArmAndElevator;
import frc.robot.commands.LowerArm;
import frc.robot.commands.ResetDriveEncoders;
import frc.robot.commands.ResetGripperEncoders;
import frc.robot.commands.SetGripperEncoderTo1;
import frc.robot.commands.ShootCube;
import frc.robot.commands.StrightenArm;
import frc.robot.commands.StrightenArmFinished;
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
  // VisionSystem visionSystem;
  Command resetCommand;
  Command strightenArm;
  Command lowerArm;
  Command cubeMode;
  Command coneMode;
  Command shootCube;
  Command armBalanceMode;

  Command tryOutPID;
  double farGoal = 0, closeGoal = 0, elevatorGoal = 0;
  AlwaysPID shablang;

  SendableChooser<String> autoChooser;
  SendableChooser<String> armChooser;

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
    // visionSystem = new VisionSystem();
    driveController = new PS4Controller(0);
    // shablang = new AlwaysPID(armSystem, elevatorSystem);


    shootCube = new ShootCube(gripperSystem);
    resetCommand = (new ResetArm(armSystem).andThen(new ArmPID(2, 2, armSystem))).alongWith((new ResetElevator(elevatorSystem)).andThen(new ElevatorPID(-10, elevatorSystem)));
    strightenArm = new ArmPID(125, 168, armSystem).alongWith(new ElevatorPID(-50, elevatorSystem));

    armBalanceMode = new ArmPID(40, 2, armSystem).alongWith(new ElevatorPID(-245.42, elevatorSystem));
    lowerArm = new ArmPID(11.038, 75.563, armSystem).alongWith(new ElevatorPID(-204, elevatorSystem));

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

    //autonomusFirstPart =  new ResetDriveEncoders(driveSystem).alongWith(new GripperPID(0.3,gripperSystem));
    //autonomusMidPart = strightenArm.andThen(shootCube.withTimeout(0.3));
    //autonomusLastPart = resetCommand;
    //autonomus = autonomusFirstPart.andThen(DriveBackwardScore.along).andThen(autonomusLastPart);
    
    // cubeMode = new CloseGripper(gripperSystem).withTimeout(1).andThen(new GripperPID(0.7, gripperSystem)).withTimeout(3).andThen(new Suck(gripperSystem));
    // coneMode = new GripperPID(0.05, gripperSystem);
    
    // shablang = resetCommand.andThen(lowerArm).andThen(cubeMode);//.andThen(resetCommand);


    driveSystem.setV_est(0);


    // tryOutPID = new ArmPID(closeGoal, farGoal, armSystem).alongWith(new ElevatorPID(-elevatorGoal, elevatorSystem));
    


    // if(driveController.getCircleButton()){
    //   // new WaitCommand(2);
    //   farGoal = 2;
    //   closeGoal = 2;
    //   elevatorGoal = 10;
    // }

    // armSystem.setDefaultCommand(tryOutPID);



    new JoystickButton(controller, PS4Controller.Button.kCross.value).toggleOnTrue(lowerArm);
    new JoystickButton(controller, PS4Controller.Button.kTriangle.value).toggleOnTrue(strightenArm);
    new JoystickButton(controller, PS4Controller.Button.kSquare.value).toggleOnTrue(armBalanceMode);
    new JoystickButton(controller, PS4Controller.Button.kCircle.value).toggleOnTrue(resetCommand);
    

    new JoystickButton(driveController, PS4Controller.Button.kSquare.value).whileTrue(new Balance(driveSystem));


    
    driveSystem.setDefaultCommand(new DriveCommand(driveSystem, driveController));


    
    
    // gripperSystem.setDefaultCommand(new CloseGripperAmp(gripperSystem));
    // gripperSystem.setDefaultCommand(new CloseGripperConst(gripperSystem));
    
    
    
    new POVButton(controller, 0).toggleOnTrue(new GripperPID(0.3, gripperSystem));
    new POVButton(controller, 180).toggleOnTrue(new GripperPID(0, gripperSystem));


    new POVButton(controller, 90).toggleOnTrue(new OpenGripper(gripperSystem));
    new POVButton(controller, 270).toggleOnTrue(new CloseGripperAmp(gripperSystem));
    
    
    new JoystickButton(controller, PS4Controller.Button.kR1.value).whileTrue(new Collect(gripperSystem));
    new JoystickButton(controller, PS4Controller.Button.kL1.value).whileTrue(new Blow(gripperSystem));
    

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
    
    SmartDashboard.putNumber("Limelight tx", limelight.TxOffset());
    SmartDashboard.putNumber("Limelight ty", limelight.TyOffset());

    SmartDashboard.putNumber("kalman",driveSystem.kalmanEstametion());
    SmartDashboard.putNumber("gian",driveSystem.kalmangain());
    SmartDashboard.putNumber("NavX", driveSystem.getPitch());
    SmartDashboard.putNumber("eest",driveSystem.eesst());

    SmartDashboard.putNumber("close distance", armSystem.getCloseJoint());
    SmartDashboard.putNumber("far distance", armSystem.getFarJoint());
    SmartDashboard.putBoolean("close limit", armSystem.getCloseSwitch());
    SmartDashboard.putBoolean("far limit", armSystem.getFarSwitch());
    SmartDashboard.putNumber("elevator hight", elevatorSystem.getHeight());
    SmartDashboard.putNumber("Pitch", driveSystem.getPitch());
    SmartDashboard.putNumber("drive encoders", driveSystem.getDistancePassedRightM());
    
    // SmartDashboard.putString("TRANSFORM PHOTON", String.format("x: %.3f, y: %.3f, z: %.3f", visionSystem.getTagDistance(), visionSystem.getTagHight(), visionSystem.getTagAngle()));

    CommandScheduler.getInstance().run();
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
    
    Command scoreState1 = new GripperPID(0.3, gripperSystem).alongWith(new ArmPID(125, 162, armSystem).alongWith(new ElevatorPID(-50, elevatorSystem)));
    Command scoreState2 = new ShootCube(gripperSystem).withTimeout(0.29).andThen(new DriveBackwardScore(driveSystem)).withTimeout(2).alongWith(resetCommand);
    
    Command balanceState1 = resetCommand;//.alongWith(new DriveForwardBalance(driveSystem));
    
    String armSelected = armChooser.getSelected();

    String autoSelected = autoChooser.getSelected();
    Command autoCommand = new ResetDriveEncoders(driveSystem);

    if(armSelected == "withArm"){
      if(autoSelected == "balance"){
        autoCommand = resetCommand;
        // autoCommand = autoCommand.andThen(balanceState1).andThen(armBalanceMode.alongWith(new Balance(driveSystem)));
        // autoCommand = autoCommand.andThen((resetCommand.withTimeout(2).andThen(armBalanceMode)).alongWith(new DriveForwardBalance(driveSystem).andThen(new Balance(driveSystem))));
      }else if(autoSelected == "communityAndBalance"){
        autoCommand = autoCommand.andThen((resetCommand.withTimeout(2).andThen(armBalanceMode)).alongWith(new DriveForwardCommunity(driveSystem).andThen(new DriveBackwardScore(driveSystem)).andThen(new Balance(driveSystem))));
      }else if(autoSelected == "scoreAndBalance"){
        autoCommand = scoreState1.withTimeout(3.15).andThen(scoreState2).withTimeout(5).andThen(new Balance(driveSystem).alongWith(armBalanceMode));
      }else if(autoSelected == "score"){
        autoCommand = scoreState1.withTimeout(3).andThen(scoreState2);
        // autoCommand = strightenArm.alongWith(new GripperPID(0.3, gripperSystem));//autoCommand.andThen(gripperState).alongWith(strightenArm);//.andThen(new ShootCube(gripperSystem).withTimeout(0.5)).withTimeout(3).andThen(resetCommand);
      }else if(autoSelected == "scoreAndCommunity"){
        autoCommand = scoreState1.withTimeout(3).andThen(scoreState2).andThen(new DriveBackwardScore(driveSystem));
      }else if(autoSelected == "scoreAndCommunityAndBalance"){
  
      }
    }else if(armSelected == "withoutArm"){
      if(autoSelected == "balance"){
        autoCommand = autoCommand.andThen(new DriveForwardBalance(driveSystem)).andThen(new Balance(driveSystem));
      }else if(autoSelected == "communityAndBalance"){
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
