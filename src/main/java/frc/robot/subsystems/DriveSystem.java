// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Arrays;
import java.util.concurrent.CountDownLatch;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.TurnToTag;

public class DriveSystem extends SubsystemBase {

  private WPI_TalonFX talonLF;
  private WPI_TalonFX talonLR;
  private WPI_TalonFX talonRR;
  private WPI_TalonFX talonRF;

  private double e_est;
  private double e_measure;
  private double v_est;

  private AHRS navie;
  
  public DriveSystem() {
    talonLF = new WPI_TalonFX(4);
    talonLR = new WPI_TalonFX(3); 
    talonRR = new WPI_TalonFX(1);
    talonRF = new WPI_TalonFX(2);

    talonLF.setInverted(true);
    talonLR.setInverted(true);
    talonRF.setInverted(false);
    talonRR.setInverted(false);

    navie = new AHRS(SPI.Port.kMXP);

    e_est = 0.05;
    e_measure = 0.01;
    v_est = 0.07;
  }

  public void drive(double R2, double L2, double turn){
    R2 = Math.min(1, Math.max(0, R2));
    L2 = Math.min(1, Math.max(0, L2));

    double turnModifyer = 0.5;
    double maxSpeed = 0.5;
    
    if (R2 > 0){
      talonLF.set(ControlMode.PercentOutput, R2 * maxSpeed + turn * maxSpeed);
      talonLR.set(ControlMode.PercentOutput, R2 * maxSpeed + turn * maxSpeed);
      talonRR.set(ControlMode.PercentOutput, R2 * maxSpeed - turn * maxSpeed);
      talonRF.set(ControlMode.PercentOutput, R2 * maxSpeed - turn * maxSpeed);
    }else if (L2 > 0){
      talonLF.set(ControlMode.PercentOutput, -L2 * maxSpeed + turn * maxSpeed);
      talonLR.set(ControlMode.PercentOutput, -L2 * maxSpeed + turn * maxSpeed);
      talonRR.set(ControlMode.PercentOutput, -L2 * maxSpeed - turn * maxSpeed);
      talonRF.set(ControlMode.PercentOutput, -L2 * maxSpeed - turn * maxSpeed);
    }else{
      if(turn > 0.05){
        talonLF.set(ControlMode.PercentOutput, turn * turnModifyer);
        talonLR.set(ControlMode.PercentOutput, turn * turnModifyer);
        talonRR.set(ControlMode.PercentOutput, -turn * turnModifyer);
        talonRF.set(ControlMode.PercentOutput, -turn * turnModifyer);
      }else if(turn < - 0.05){
        talonLF.set(ControlMode.PercentOutput, turn * turnModifyer);
        talonLR.set(ControlMode.PercentOutput, turn * turnModifyer);
        talonRR.set(ControlMode.PercentOutput, -turn * turnModifyer);
        talonRF.set(ControlMode.PercentOutput, -turn * turnModifyer);
      }else{
        talonLF.set(0);
        talonLR.set(0);
        talonRR.set(0);
        talonRF.set(0);
      }
    }
  }

  public void tankDrive(double right, double left){
    talonLF.set(ControlMode.PercentOutput, left);
    talonLR.set(ControlMode.PercentOutput, left);
    talonRR.set(ControlMode.PercentOutput, right);
    talonRF.set(ControlMode.PercentOutput, right);
  }

  public void stop(){
    talonLF.set(ControlMode.PercentOutput, 0);
    talonLR.set(ControlMode.PercentOutput, 0);
    talonRR.set(ControlMode.PercentOutput, 0);
    talonRF.set(ControlMode.PercentOutput, 0);
  }

  public double getDistancePassedLeftM() {
    return talonLF.getSelectedSensorPosition(); // Constants.TALON_FX_PPR / Constants.DRIVE_GEAR_RATIO * Constants.DRIVE_WHEEL_CIRCUMEFERENCE_M;
  }

  public double getDistancePassedRightM() {
    return talonRF.getSelectedSensorPosition(); // Constants.TALON_FX_PPR / Constants.DRIVE_GEAR_RATIO * Constants.DRIVE_WHEEL_CIRCUMEFERENCE_M;
  }

  /*
  public double getDistancePassedLeftM() {
    return (talonLF.getSelectedSensorPosition() + talonLR.getSelectedSensorPosition()) / 2.0; // Constants.TALON_FX_PPR / Constants.DRIVE_GEAR_RATIO * Constants.DRIVE_WHEEL_CIRCUMEFERENCE_M;
  }

  public double getDistancePassedRightM() {
    return (talonRF.getSelectedSensorPosition() + talonRR.getSelectedSensorPosition()) / 2.0; // Constants.TALON_FX_PPR / Constants.DRIVE_GEAR_RATIO * Constants.DRIVE_WHEEL_CIRCUMEFERENCE_M;
  }
 */

  public double getDistancePassedM() {
    return (getDistancePassedLeftM() + getDistancePassedRightM()) / 2.0;
  }

  public void resetEncoders() {
    talonLF.setSelectedSensorPosition(0);
    talonRF.setSelectedSensorPosition(0);
    talonLR.setSelectedSensorPosition(0);
    talonRR.setSelectedSensorPosition(0);
  }

  public double getPitch(){
    return navie.getPitch();
  }

  public double kalmangain() {  
    return (e_est / (e_est + e_measure));
  }
  //delta of the derivatives
  //sets count to 0

  public double kalmanEstametion() {
    return v_est = v_est + kalmangain() * (getPitch() - v_est);
  }

  public double eesst() {
   if(e_est > 0.015) {
    return e_est =  (1 - kalmangain()) * e_est;
   }
    return e_est = 0.013;
  }

  public void setV_est(double valueToSet){
    v_est = valueToSet;
  }
}
