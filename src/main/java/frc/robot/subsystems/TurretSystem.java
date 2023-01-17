// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TurretSystem extends SubsystemBase {

  TalonFX motor;

  public TurretSystem() {
    motor = new TalonFX(0);
  }

  public void rotate(double speed){
    motor.set(ControlMode.PercentOutput, speed);
  }

  public double getAngle(){
    return motor.getSelectedSensorPosition()*360/Constants.TALON_FX_PPR;
  }

  public void stop(){
    motor.set(ControlMode.PercentOutput, 0);
  }

}
