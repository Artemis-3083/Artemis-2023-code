// CopyrightSpin (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxRelativeEncoder.Type;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class GripperSystem extends SubsystemBase {
  
  private CANSparkMax motor;
  private CANSparkMax motorWheels;

  public GripperSystem() {
    motor = new CANSparkMax(7, MotorType.kBrushed);
    motorWheels = new CANSparkMax(9, MotorType.kBrushless);
    motor.setInverted(false);
    motorWheels.setInverted(true);
    // resetEncoder();
  }

  public void move(double speed){
    motor.set(speed);
  }

  public void moveWheels(double speed){
    motorWheels.set(speed);
  }

  public void stop(){
    motor.set(0);
    motorWheels.set(0);
  }
  
  public double getEncoder(){ 
    return motor.getEncoder(Type.kQuadrature, 8192).getPosition() * Constants.GRIPPER_1_PER_PULSE;
  }

  public void resetEncoder(){
    motor.getEncoder(Type.kQuadrature, 8192).setPosition(0);
  }

  @Override
  public void periodic() {
    if(getEncoder() > 10){
      motor.getEncoder(Type.kQuadrature, 8192).setPosition(0);
    }
    SmartDashboard.putNumber("Gripper encoder", getEncoder());
  }
}
