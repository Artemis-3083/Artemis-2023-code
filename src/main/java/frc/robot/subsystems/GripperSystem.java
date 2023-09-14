// CopyrightSpin (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxRelativeEncoder.Type;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class GripperSystem extends SubsystemBase {
  
  private CANSparkMax motor;
  private CANSparkMax motorWheels;
  private RelativeEncoder encoder;

  public GripperSystem() {
    motor = new CANSparkMax(7, MotorType.kBrushed);
    motorWheels = new CANSparkMax(9, MotorType.kBrushless);
    encoder = motor.getEncoder(Type.kQuadrature, 8192);
    motor.setInverted(false);
    motorWheels.setInverted(true);
  }

  public void move(double speed){
    motor.set(speed);
  }

  public void moveWheels(double speed){
    motorWheels.set(speed);
  }

  public void stopMotor(){
    motor.set(0);
  }

  public void stopMotorWheels(){
    motorWheels.set(0);
  }

  public void stop(){
    stopMotor();
    stopMotorWheels();
  }
  
  public double getCurrent(){
    return motor.getOutputCurrent();
  }

  public double getWheelsCurrent(){
    return motorWheels.getOutputCurrent();
  }

  public double getEncoder(){ 
    return encoder.getPosition() * Constants.GRIPPER_1_PER_PULSE;
  }

  public void resetEncoder(){
    encoder.setPosition(0);
  }

  public void setEncoder(double position){
    encoder.setPosition(position);
  }

  @Override
  public void periodic() {
    if(getEncoder() > 10){
      encoder.setPosition(0);
    }
    SmartDashboard.putNumber("Gripper encoder", getEncoder());
    SmartDashboard.putNumber("Gripper speed", motor.get());
    SmartDashboard.putNumber("Gripper open current", motor.getOutputCurrent());
    SmartDashboard.putNumber("Gripper vroom current", motorWheels.getOutputCurrent());
  }
}
