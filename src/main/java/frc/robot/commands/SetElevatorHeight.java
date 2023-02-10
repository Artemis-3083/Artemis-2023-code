// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSystem;

public class SetElevatorHeight extends CommandBase {
    private static final double KP = 0.3;
    private static final double KI = 0.06;
    private static final double KD = 0;
    private static final double PERIOD_SEC = 0.02;
    private final PIDController controller;

  public ElevatorSystem elevator;
  public double height;

  /** Creates a new SetElevatorHeight. */
  public SetElevatorHeight(double height,ElevatorSystem elevator) {
    controller = new PIDController(KP, KI, KD, PERIOD_SEC);
    this.elevator=elevator;
    this.height=height;
    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

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
    return false;
  }
}
