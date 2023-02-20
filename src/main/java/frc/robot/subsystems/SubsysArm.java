// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SubsysArm extends SubsystemBase {
  private CANSparkMax cloesSpark;
  private CANSparkMax farSpark;
  private Encoder closeEncoder;
  private Encoder farEncoder;
  private double speed = 0;
  private double wdith;
  private double disstance;
  private double hight;

  public SubsysArm() {
    cloesSpark = new CANSparkMax(0, MotorType.kBrushless);
    farSpark = new CANSparkMax(0, MotorType.kBrushless);
    closeEncoder = new Encoder(0, 0);
    farEncoder = new Encoder(0, 0);
  }

  public void moveBoth(double speed) {
    cloesSpark.set(speed);
    farSpark.set(speed);
  }

  public void stop() {
    cloesSpark.set(0);
    farSpark.set(0);
  }

  public double closeEncoder() {
    return 360  * closeEncoder.get(); //change to right calcuation
  }

  public double farEncoderget() {
    return 360 * farEncoder.get(); //change to right calcuation
  }

  public void closeSpark(double speed) {
    cloesSpark.set(speed);
  } 

  public void farSpark(double speed) {
    farSpark.set(speed);
  }
 
  public double distance(double disstance,double hight) {
    return Math.sqrt(Math.pow(0.5,2) + Math.pow(0.4,2) - 2 * 0.5 * 0.4 * Math.cos(farEncoderget()));
  }     

  public double hight() {
    return hight;
  }

  public double length() {
    return disstance;//from the game piece
  }
}
