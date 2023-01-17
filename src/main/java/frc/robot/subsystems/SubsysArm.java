// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SubsysArm extends SubsystemBase {
  
  CANSparkMax armspark;

  public SubsysArm() {
    armspark = new CANSparkMax(0, MotorType.kBrushless);
    armspark.getEncoder().getPosition();
  }

  private double routation = armspark.getEncoder().getPosition();
  
  private int gear = 1;
  private double speed = 0;
  private double deg = 0;

  public void motorspeedincrees() {
    if(speed <= 1)
    speed = speed + 0.05;
    
    armspark.set(speed);
  }

  
  public void motorspeeddecreas() {
    speed = speed - 0.05;
    armspark.set(speed);
  }

  public void motorstop() {
    armspark.set(0);
  }

  public double degries() {
    deg = (routation * 360) / gear;
    return deg; 
  }

  public double spied() {
    return speed;
  }
}
