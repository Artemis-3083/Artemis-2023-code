// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LimelightSystem extends SubsystemBase {

  private NetworkTable limelightTable;
  
  public LimelightSystem() {
    limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
  }

  public double TxOffset(){
    return limelightTable.getEntry("tx").getDouble(0);
  }

  public double TyOffset(){
    return limelightTable.getEntry("ty").getDouble(0);
  }
  
  //DISTANCE FORMULA = (TARGET_HEIGHT-LIMELIGHT_HEIGHT) / Math.tan(Math.toRadians(LIMELIGHT_ANGLE+TyOffset));

  public double getHighReflectiveDistance(){
    return (Constants.REFLECTIVE_HIGH_HEIGHT_M - Constants.LIMELIGHT_HEIGHT_M) / Math.tan(Math.toRadians(Constants.LIMELIGHT_ANGLE + TyOffset()));
  }

  public double getLowReflectiveDistance(){
    return (Constants.REFLECTIVE_LOW_HEIGHT_M - Constants.LIMELIGHT_HEIGHT_M) / Math.tan(Math.toRadians(Constants.LIMELIGHT_ANGLE + TyOffset()));
  }

  public double getConeDistance(){
    return (Constants.CONE_HEIGHT_M - Constants.LIMELIGHT_HEIGHT_M) / Math.tan(Math.toRadians(Constants.LIMELIGHT_ANGLE + TyOffset()));
  }

  public double getCubeDistance(){
    return (Constants.CUBE_HEIGHT_M - Constants.LIMELIGHT_HEIGHT_M) / Math.tan(Math.toRadians(Constants.LIMELIGHT_ANGLE + TyOffset()));
  }

  public boolean areVisableTarget(){
    if(limelightTable.getEntry("tv").getInteger(0) == 1){
      return true;
    }
    return false;
  }

  public void setPipeline(int pipeline){
    limelightTable.getEntry("pipeline").setNumber(pipeline);
  }

  public double getPipeline(){
    /*
     * pipeline index --> meaning
     * 0 --> default value, error
     * 1 --> reflective tape
     * 2 --> colored shapes
     * 3 --> apriltags
    */
    return limelightTable.getEntry("getpipe").getDouble(0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("limelight pipeline", getPipeline());
  }

}
