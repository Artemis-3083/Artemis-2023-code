// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Arrays;

import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.kauailabs.navx.frc.AHRS;

import org.opencv.aruco.EstimateParameters;

import edu.wpi.first.networktables.DoubleArrayEntry;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PLEASE_GOD_HELPME_END_MY_SUFFERING<measurmantArray> extends SubsystemBase {

    private WPI_Pigeon2 pigeon;
    private double angle;
    private double exp;
    private AHRS navie;
    private double eest;// current estemation
    private double Eest_t; // previoes estemation
    private double kg;// kalman gain
    private double emeasurmant = 0.2;// the navx musearing error
    private double imea;// taken when ramp hit 15 deg
    private double[] measurmantArray;
    private double[] weightsArray;
    private int i = 0;

    public PLEASE_GOD_HELPME_END_MY_SUFFERING() {
        angle = 0; // change to angle
        exp = Math.exp(1);
        navie = new AHRS(SPI.Port.kMXP);
        pigeon = new WPI_Pigeon2(0);
        measurmantArray = new double[100];
        weightsArray = new double[100];
        eest = eest;
    }

    public double getPitch() {
        return pigeon.getPitch();
    }

    public double[] addToPitchArray() {
        for (int i = 0; i < 100; i++) {
            if (measurmantArray[i] == 0) {
                measurmantArray[i] = getPitch();
                return measurmantArray;
            }
        }
        return measurmantArray;
    }

    public void addToWeightsArray() {
        for (int i = 0; i < 100; i++) {
            if (weightsArray[i] == 0) {
                weightsArray[i] = measurmant();
                return;
            }
        }
    }

    public double measurmant() {
        exp += 0.02;
        return Math.acos(exp * (1 - Math.cos(getPitch()) / 2)); // getPitch() --> angle
    }

    public double[] predict() {
        double c = 1;
        int q = 15;
        double weighted_error = 0.0;
        int n = (int) measurmantArray[0];
        double[] y_pred = new double[n];
        double[] epsilon = new double[n];
        int window;
        Arrays.fill(y_pred, 0.0);
        Arrays.fill(epsilon, 0.0);
        for (int t = 0; t < n; t++) {
            n = (int) measurmantArray[t];
            epsilon[t] = y_pred[t] - c;
            weighted_error = 0.0;
            window = Math.min(t + 1, q);
            for (int i = 1; i <= window; i++) {
                weighted_error += weightsArray[i - 1] * epsilon[t - i];
            }
            y_pred[t] = c + weighted_error; // weight error prediction
        }
        return y_pred;// I ALONE AM THE HONORED ONE UPON THE HEAVENS AND THE EARTH AND EVERYTHING IN
                      // BETWEEN, I AM GOD'S FAVORITE AND CHOSEN IDOL AND ALL SHALL BOW BEFORE ME
    }

    public double kalmangain(int count,double [] y_pred) {
        kg = y_pred[count] / (emeasurmant + y_pred[count]);// nigga what is this
        return kg;
    }

    public double kalmanEstametion(int count,double [] y_pred) {
        if (count == 0) {
            eest = y_pred[0] + kalmangain(count,y_pred) * (measurmant() - y_pred[0]);
            count++;
            return eest / 10;
        } else {
            eest = y_pred[count - 1] + kalmangain(count,y_pred) * (measurmant() - y_pred[count - 1]);
            count++;
            return eest / 100;  
        }
    }

}
