// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {
  private CANSparkMax closeJoint;
  private CANSparkMax farJoint;

  public ArmSubsystem() {
    closeJoint = new CANSparkMax(0, MotorType.kBrushless);
    farJoint = new CANSparkMax(0, MotorType.kBrushless);
  }

  public void stopFarJoint() {
    farJoint.set(0);
  }

  public void stopCloseJoint() {
    closeJoint.set(0);
  }

  public double getCloseJoint() {
    return closeJoint.getEncoder().getPosition() / Constants.SPARK_MAX_PPR / Constants.ARM_CLOSE_MOTOR_GEAR_RATIO * Constants.ARM_CLOSE_WHEEL_CIRCUMEFERENCE_M;
  }
  
  public double getFarJoint() {
    return farJoint.getEncoder().getPosition() / Constants.SPARK_MAX_PPR / Constants.ARM_FAR_MOTOR_GEAR_RATIO * Constants.ARM_FAR_WHEEL_CIRCUMEFERENCE_M;
  }

  public void moveCloseJoint(double speed) {
    closeJoint.set(speed);
  } 

  public void moveFarJoint(double speed) {
    farJoint.set(speed);
  }
 
  public double distance(double disstance,double hight) { //??????
    return Math.sqrt(Math.pow(0.5,2) + Math.pow(0.4,2) - 2 * 0.5 * 0.4 * Math.cos(getFarJoint()));
  }     
}
