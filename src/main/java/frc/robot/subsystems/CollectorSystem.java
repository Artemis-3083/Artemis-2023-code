// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CollectorSystem extends SubsystemBase {
  
  CANSparkMax right, left;
  Solenoid piston;

  public CollectorSystem() {
    right = new CANSparkMax(0, MotorType.kBrushless);
    left = new CANSparkMax(0, MotorType.kBrushless);
    left.setInverted(true);
    piston = new Solenoid(PneumaticsModuleType.CTREPCM, 0);
  }

  public void collect(){
    right.set(1);
    left.set(1);
  }

  public void release(){
    right.set(-1);
    left.set(-1);
  }

  public void stop(){
    right.set(0);
    left.set(0);
  }

  public void grabOrRelease(){
    piston.toggle();
  }
}
