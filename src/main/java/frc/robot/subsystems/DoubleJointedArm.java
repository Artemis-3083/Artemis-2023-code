// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DoubleJointedArm extends SubsystemBase {
  
  private CANSparkMax motor1;
  private CANSparkMax motor2;
  private double joint1MinAngle;
  private double joint1MaxAngle;
  private double joint2MinAngle;
  private double joint2MaxAngle;

  public DoubleJointedArm(double joint1Min, double joint1Max, double joint2Min, double joint2Max) {
    motor1 = new CANSparkMax(0, MotorType.kBrushless);
    motor2 = new CANSparkMax(1, MotorType.kBrushless);
    joint1MinAngle = joint1Min;
    joint1MaxAngle = joint1Max;
    joint2MinAngle = joint2Min;
    joint2MaxAngle = joint2Max;
  }

  public void moveArm(double desiredJoint1Angle, double desiredJoint2Angle) {
    
    // Read current joint angles from sensors
    double currentJoint1Angle = readJoint1Angle();
    double currentJoint2Angle = readJoint2Angle();

    // Convert desired joint angles to motor speed values
    double joint1Speed = angleToSpeed(desiredJoint1Angle, joint1MinAngle, joint1MaxAngle);
    double joint2Speed = angleToSpeed(desiredJoint2Angle, joint2MinAngle, joint2MaxAngle);

    // Adjust motor movements based on current and desired joint angles
    while (Math.abs(currentJoint1Angle - desiredJoint1Angle) > 0.1 || Math.abs(currentJoint2Angle - desiredJoint2Angle) > 0.1) {
      currentJoint1Angle = readJoint1Angle();
      currentJoint2Angle = readJoint2Angle();
      double speed1 = joint1Speed * (desiredJoint1Angle - currentJoint1Angle < 0 ? -1 : 1);
      double speed2 = joint2Speed * (desiredJoint2Angle - currentJoint2Angle < 0 ? -1 : 1);
      motor1.set(speed1);
      motor2.set(speed2);
    }

    // Stop motors when arm has reached desired position
    motor1.set(0);
    motor2.set(0);
  }

    private double readJoint1Angle() {
        // Code to read current angle of joint 1 from sensor
        return 0.0;
    }

    private double readJoint2Angle() {
        // Code to read current angle of joint 2 from sensor
        return 0.0;
    }

    private double angleToSpeed(double angle, double minAngle, double maxAngle) {
        double range = maxAngle - minAngle;
        double angleRange = angle - minAngle;
        double speedRange = 1.0;
        double speed = (angleRange / range) * speedRange;
        return speed;
    }
private double desiredangle(){
// Assume the x and y coordinates of the cone are stored in variables called coneX and coneY

// Calculate the distance between the arm base and the cone
double armToConeDistance = Math.sqrt(coneX*coneX + coneY*coneY);

// Calculate the angle of the first joint using the arctangent function
double angle1 = Math.atan2(coneY, coneX);

// Calculate the distance between the end effector and the cone using the Pythagorean theorem
double gripperToConeDistance = armToConeDistance - armLength2;

// Calculate the angle of the second joint using the law of cosines
double angle2 = Math.acos((gripperToConeDistance*gripperToConeDistance - armLength1*armLength1 - armLength2*armLength2)/(-2*armLength1*armLength2));

// Convert the angles from radians to degrees
angle1 = Math.toDegrees(angle1);
angle2 = Math.toDegrees(angle2);
}
}
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
