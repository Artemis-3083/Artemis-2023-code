// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveSystem extends SubsystemBase {

  private WPI_TalonFX talonLF;
  private WPI_TalonFX talonLR;
  private WPI_TalonFX talonRR;
  private WPI_TalonFX talonRF;

  private AHRS navie;
  
  public DriveSystem() {

    talonLF = new WPI_TalonFX(2);
    talonLR = new WPI_TalonFX(1);
    talonRR = new WPI_TalonFX(3);
    talonRF = new WPI_TalonFX(4);

    talonLF.setInverted(false);
    talonLR.setInverted(false);
    talonRF.setInverted(true);
    talonRR.setInverted(true);

    navie = new AHRS(SPI.Port.kMXP);

    navie.reset();
  }

  public void drive(double R2, double L2, double turn){
    R2 = Math.min(1, Math.max(0, R2));
    L2 = Math.min(1, Math.max(0, L2));
    
    if (R2 > 0){
      talonLF.set(R2 * 0.5 - turn * 0.5);
      talonLR.set(R2 * 0.5 - turn * 0.5);
      talonRR.set(R2 * 0.5 + turn * 0.5);
      talonRF.set(R2 * 0.5 + turn * 0.5);
    }else if (L2 > 0){
      talonLF.set(-L2 * 0.5 - turn * 0.5);
      talonLR.set(-L2 * 0.5 - turn * 0.5);
      talonRR.set(-L2 * 0.5 + turn * 0.5);
      talonRF.set(-L2 * 0.5 + turn * 0.5);
    }else{
      if(turn > 0.05){
        talonLF.set(-turn);
        talonLR.set(-turn);
        talonRR.set(turn);
        talonRF.set(turn);
      }else if(turn < - 0.05){
        talonLF.set(-turn);
        talonLR.set(-turn);
        talonRR.set(turn);
        talonRF.set(turn);
      }else{
        talonLF.set(0);
        talonLR.set(0);
        talonRR.set(0);
        talonRF.set(0);
      }
    }
  }

  public void stop(){
    talonLF.set(0);
    talonLR.set(0);
    talonRR.set(0);
    talonRF.set(0);
  }

  public double getDistancePassedLeftM() {
    return talonLF.getSelectedSensorPosition() / Constants.TALON_FX_PPR /*/ Constants.DRIVE_GEAR_RATIO * Constants.DRIVE_WHEEL_CIRCUMEFERENCE_M*/;
  }

  public double getDistancePassedRightM() {
    return talonRF.getSelectedSensorPosition() / Constants.TALON_FX_PPR /*/ Constants.DRIVE_GEAR_RATIO * Constants.DRIVE_WHEEL_CIRCUMEFERENCE_M*/;
  }

  public double getDistancePassedM() {
    return (getDistancePassedLeftM() + getDistancePassedRightM()) / 2.0;
  }

  public void resetEncoders() {
    talonLF.setSelectedSensorPosition(0);
    talonRF.setSelectedSensorPosition(0);
  }

  public double getPitch(){
    return navie.getPitch();
  }
}
