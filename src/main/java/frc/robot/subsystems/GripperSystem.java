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

  public GripperSystem() {
    motor = new CANSparkMax(7, MotorType.kBrushed);
    motor.getEncoder(Type.kQuadrature, 8192).setPosition(0);
  }

  public void move(double speed){
    if(speed < 0.5 && speed > -0.5){
      if(speed > 0){
        motor.set(0.5);
      }else{
        motor.set(-0.5);
      }
    }
    motor.set(speed);
  }

  public void stop(){
    motor.set(0);
  }
  
  public double getEncoder(){
    return motor.getEncoder(Type.kQuadrature, 8192).getPosition() * Constants.GRIPPER_1_PER_PULSE;
  }

  @Override
  public void periodic() {
      SmartDashboard.putNumber("Gripper encoder", getEncoder());
  }
}
