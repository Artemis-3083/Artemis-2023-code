// CopyrightSpin (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CollectorSystem extends SubsystemBase {
  
  private CANSparkMax leftSpin; 
  private CANSparkMax rightSpin;
  private CANSparkMax grip;

  public CollectorSystem() {
    leftSpin = new CANSparkMax(20, MotorType.kBrushless);
    rightSpin = new CANSparkMax(21, MotorType.kBrushless);
    leftSpin.setInverted(true);
    grip = new CANSparkMax(22, MotorType.kBrushless);
  }

  public void collect(){
    rightSpin.set(1);
    leftSpin.set(1);
  }

  public void release(){
    rightSpin.set(-1);
    leftSpin.set(-1);
  }

  public void stopSpin(){
    rightSpin.set(0);
    leftSpin.set(0);
  }

  public void open(){
    grip.set(0.5);
  }

  public void close(){
    grip.set(0.5);
  }

  public void stopGrip(){
    grip.set(0);
  }
}
*/