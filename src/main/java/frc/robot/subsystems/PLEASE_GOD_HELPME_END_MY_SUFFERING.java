// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PLEASE_GOD_HELPME_END_MY_SUFFERING extends SubsystemBase {

    private double angle;
    private double exp;
    private AHRS navie;
  
    public PLEASE_GOD_HELPME_END_MY_SUFFERING() {
        angle = 0; //change to angle
        exp = Math.exp(1);
        navie = new AHRS(SPI.Port.kMXP);
    }

    public double yes(){
        return Math.acos(exp*(1-Math.cos(getPitch())/2)) * 0.05; // getPitch() --> angle
    }

    public double getPitch(){
        return navie.getRoll();
    }
}
