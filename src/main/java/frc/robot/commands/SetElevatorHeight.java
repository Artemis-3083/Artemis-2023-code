// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSystem;

public class SetElevatorHeight extends CommandBase {
  private static final double KP = 0.1;
  private static final double KI = 0;
  private static final double KD = 0.0;
  private static final double PERIOD_SEC = 0.02;
  private static final double TIME_STABILIZED_SEC = 1;

  private final PIDController controller;

  public ElevatorSystem elevator;
  public double height;
  private double atSetpointStartTime;

  /** Creates a new SetElevatorHeight. */
  public SetElevatorHeight(double height,ElevatorSystem elevator) {
    controller = new PIDController(KP, KI, KD, PERIOD_SEC);
    controller.setTolerance(0.01);

    this.elevator=elevator;
    this.height=height;
    addRequirements(elevator);

    SmartDashboard.putData("SetElevatorHeight.PID", controller);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    atSetpointStartTime = -1;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double processVariable = elevator.getHeight();
    double output = controller.calculate(processVariable, height);
    output = MathUtil.clamp(output, -1, 1);
    elevator.elevatorSetSpeed(output);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevator.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (controller.atSetpoint()) {
      if (atSetpointStartTime <= 0) {
        atSetpointStartTime = RobotController.getFPGATime();
      } else {
        return RobotController.getFPGATime() - atSetpointStartTime >= TIME_STABILIZED_SEC * 10e6;
      }
    } else {
      atSetpointStartTime = -1;
    }

    return false;
  }
}
