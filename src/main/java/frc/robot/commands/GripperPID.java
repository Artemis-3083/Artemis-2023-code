// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.GripperSystem;

public class GripperPID extends CommandBase {

  
  private static final double KP = 0.3;
  private static final double KI = 0;
  private static final double KD = 0.4;
  private static final double PERIOD_SEC = 0.02;
  private static final double TIME_STABILIZED_SEC = 1;

  private final PIDController pidController;

  private double atSetpointStartTime;
  
  private final double goal;
  private GripperSystem gripperSystem;

  public GripperPID(double goal, GripperSystem gripperSystem) {
    this.goal = goal;
    this.gripperSystem = gripperSystem;
    pidController = new PIDController(KP, KI, KD, PERIOD_SEC);
    pidController.setTolerance(0.01);
    addRequirements(gripperSystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    atSetpointStartTime = -1;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double output = pidController.calculate(gripperSystem.getEncoder(), goal);
    if (gripperSystem.getEncoder() > 0){
      output = MathUtil.clamp(output, 0.5, 1);
    }else{
      output = MathUtil.clamp(output, -1, -0.5);
    }
    gripperSystem.move(output);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    gripperSystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(pidController.atSetpoint()){
      if(atSetpointStartTime <= 0){
        atSetpointStartTime = RobotController.getFPGATime();
      }else{
        return RobotController.getFPGATime() - atSetpointStartTime >= TIME_STABILIZED_SEC * 10e6;
      }
    }else{
      atSetpointStartTime = -1;
    }
    return false;
    //return gripperSystem.getEncoder() > goal - 5 && gripperSystem.getEncoder() < goal + 5;
  }
}
