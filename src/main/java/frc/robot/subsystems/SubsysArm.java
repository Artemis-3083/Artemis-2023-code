// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.sim.ArmSystemSim;

public class SubsysArm extends SubsystemBase {
    private CANSparkMax cloesSpark;
    private CANSparkMax farSpark;
    private Encoder closeEncoder;
    private Encoder farEncoder;
    private double speed = 0;
    private double wdith;

    private final ArmSystemSim sim;

    public SubsysArm() {
        cloesSpark = new CANSparkMax(0, MotorType.kBrushless);
        farSpark = new CANSparkMax(1, MotorType.kBrushless);
        //closeEncoder = new Encoder(0, 0);
        //farEncoder = new Encoder(0, 0);

        if (Robot.isSimulation()) {
            sim = new ArmSystemSim();
        } else {
            sim = null;
        }
    }

    public void moveBoth(double speed) {
        cloesSpark.set(speed);
        farSpark.set(speed);
    }

    public void stop() {
        cloesSpark.set(0);
        farSpark.set(0);
    }

    public double getCloseEncoderAngleDegrees() {
        if (Robot.isSimulation()) {
            return sim.getFirstJointAngleDegrees();
        } else {
            return  wdith*closeEncoder.get(); //change to right calcuation
        }
    }

    public double getFarEncoderAngleDegrees() {
        if (Robot.isSimulation()) {
            return sim.getSecondJointAngleDegrees();
        } else {
            return wdith * farEncoder.get(); //change to right calcuation
        }
    }

    public void closeSpark(double speed) {
        cloesSpark.set(speed);
    }

    public void farSpark(double speed) {
        farSpark.set(speed);
    }


    @Override
    public void periodic() {
        SmartDashboard.putNumber("Arm.FirstJoint.Angle", getCloseEncoderAngleDegrees());
        SmartDashboard.putNumber("Arm.SecondJoint.Angle", getFarEncoderAngleDegrees());
    }

    @Override
    public void simulationPeriodic() {
        sim.update(cloesSpark.get(), farSpark.get(), 0.02);
    }
}
