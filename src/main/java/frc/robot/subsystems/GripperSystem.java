// CopyrightSpin (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.EncoderType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxRelativeEncoder.Type;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GripperSystem extends SubsystemBase {
  
  private CANSparkMax motor;
  Encoder encoder;

  public GripperSystem() {
    motor = new CANSparkMax(7, MotorType.kBrushed);
    
  }

  public void open(){
    motor.set(1);
  }

  public void close(){
    motor.set(-1);
  }

  public void stop(){
    motor.set(0);
  }
  
  public double getEncoder(){
    return motor.getEncoder(Type.kQuadrature, 8192).getPosition();
  }

  @Override
  public void periodic() {
      SmartDashboard.putNumber("Gripper encoder", getEncoder());
  }
}
