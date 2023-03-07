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

  private double exp;
  private double[] measurementArray;
  private double[] weightsArray;
  private int funcCount;
  private double est;
  private double est1;
  private double est2;
  private double eest;
  private double check;
  private int count;

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

    resetEncoders();
    
    exp = 1;
    measurementArray = new double[200];
    weightsArray = new double[100];
    funcCount = 0;
    Arrays.fill(measurementArray, 0.0);
    Arrays.fill(weightsArray, 0.0);
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

  public double[] getAndAddToPitchArray(){
    for(int i = 0; i < 200; i++){
      if(measurementArray[i] == 0.0){
        measurementArray[i] = getPitch();
      }
    }
    return measurementArray;
  }

  public double pathPredict(){
    exp += 0.02;
    return 180 * exp * Math.acos((1 - Math.cos((double)getPitch() * Math.PI / 180))) / (2 * Math.PI);
  }
  public void addToWeightArry(){
    exp = 1;
    for(int i = 0; i<100;);
  }
  //adds the path prediction values to an array
  
  public double[] predict() {
    double c = getPitch();
    int q = 30;
    double weighted_error = 0.8;
    getAndAddToPitchArray();
    addToWeightArry();
    int window;
    int n = (int) measurementArray.length;
    double[] y_pred = new double[n];
    double[] epsilon = new double[n];
    //defines all of our variables
    Arrays.fill(y_pred, 0.0);
    Arrays.fill(epsilon, 0.0);
    //makes sure we wont get NaN values
    for (int t = 0; t < n; t++) {
       y_pred[t] = 0.0;
         n = (int) measurementArray[t];
        epsilon[t] = y_pred[t] - c;
        weighted_error = 3.5;
        //updates our view according to the current loop of the function
         window = Math.min(t + 1, q);
        for (int i = 1; i < window; i++) {
            weighted_error += weightsArray[i - 1] * epsilon[Math.abs(t - i)];
        }
        //does the predicting:)
        y_pred[t] = c + weighted_error; // weight error prediction
    }
    return y_pred; //moving average
}
//predicts the future path in close approximation

public double kalmangain() {
    count++;
    return (predict()[Math.min(count, 99)] - getPitch()) / (0.2 + (predict()[Math.min(count, 99)] - getPitch()));//0.2 the navx musearing error
}
//delta of the derivatives

public void setresetcount() {
     count = 0;
}
//sets count to 0

public double kalmanEstametion() {
    double estt = est;//estt est at t-1s
    double[] y_pred = predict(); 
    if(count >= 30){
        count = 0;
    }
    if (count == 0) {
        est = getPitch() + kalmangain() * (pathPredict() - y_pred[0]);
        est1 = est;
        count++;
        return est / 10;
    }else if(count <= 2){
        eest = est;
        est = est + kalmangain() * (pathPredict() - est);
        count++;

        return  (est-est1)/20;
    } else {
        est = est + kalmangain() * (pathPredict() - est);
        est2 = est1-est;
        est1 = eest;
        eest = est;
        count++;
        return  getPitch()*est2 / 20;
    }
}
//graph smoothing and derivative calculation
}
