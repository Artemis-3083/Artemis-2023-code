// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {

  private DigitalInput closeSwitch;
  private DigitalInput farSwitch;

  private TalonFX closeJoint;
  private CANSparkMax farJoint;

  public ArmSubsystem() {
    closeJoint = new TalonFX(8);
    farJoint = new CANSparkMax(6, MotorType.kBrushless);
    closeSwitch = new DigitalInput(7);
    farSwitch = new DigitalInput(6);
    farJoint.setInverted(false);
  }

  public boolean getCloseSwitch(){
    return !closeSwitch.get();
  }

  public boolean getFarSwitch(){
    return !farSwitch.get();
  }

  public void resetCLose(){
    closeJoint.setSelectedSensorPosition(0);
  }

  public void resetFar(){
    farJoint.getEncoder().setPosition(0);
  }

  public void stopFarJoint() {
    farJoint.set(0);
  }

  public void stopCloseJoint() {
    closeJoint.set(ControlMode.PercentOutput, 0);
  }

  public double getCloseJoint() {
    return closeJoint.getSelectedSensorPosition();// * Constants.ARM_ANGLE_PER_PULSE;// / Constants.SPARK_MAX_PPR / Constants.ARM_CLOSE_MOTOR_GEAR_RATIO * Constants.ARM_CLOSE_WHEEL_CIRCUMEFERENCE_M;
  }
  
  public double getFarJoint() {
    return farJoint.getEncoder().getPosition() * Constants.FAR_JOINT_ANGLE_PER_PULSE;// / Constants.SPARK_MAX_PPR / Constants.ARM_FAR_MOTOR_GEAR_RATIO * Constants.ARM_FAR_WHEEL_CIRCUMEFERENCE_M;
  }

  public double getArmLength(){
    return getFarJoint() + getCloseJoint();
  }

  public void moveCloseJoint(double speed) {
    closeJoint.set(ControlMode.PercentOutput, speed);
  } 

  public void moveFarJoint(double speed) {
    farJoint.set(speed);
  }
 
  public double distance(double disstance,double hight) { //??????
    return Math.sqrt(Math.pow(0.5,2) + Math.pow(0.4,2) - 2 * 0.5 * 0.4 * Math.cos(getFarJoint()));
  }

  @Override
  public void periodic() {
    if(getCloseSwitch()){
      resetCLose();
    }if(getFarSwitch()){
      resetFar();
    }
  }
}
