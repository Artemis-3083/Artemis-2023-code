// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

public final class Constants {
    
    //PPR
    public static final double TALON_FX_PPR = 2048;
    public static final double SPARK_MAX_PPR = 42;

    //ARM
    public static final double ARM_FAR_MOTOR_GEAR_RATIO = 1/50;
    public static final double ARM_CLOSE_MOTOR_GEAR_RATIO = 1/50;
    public static final double ARM_FAR_WHEEL_CIRCUMEFERENCE_M = 1;
    public static final double ARM_CLOSE_WHEEL_CIRCUMEFERENCE_M = 1;
    // public static final double FAR_JOINT_ANGLE_PER_PULSE = 238.1 / 141.854; //219.2 pulses //27.428
    public static final double FAR_JOINT_ANGLE_PER_PULSE = 60.0 / 27.428;
    // public static final double CLOSE_JOINT_ANGLE_PER_PULSE = 180.0 / 118866;
    public static final double CLOSE_JOINT_ANGLE_PER_PULSE = 90.0 / 60855.0;

    //ELEVATOR
    public static final double ELEVATOR_WHEEL_CIRCUMEFERENCE_M = 1.432 * 25.4 / 1000 * 2 * Math.PI;
    public static final double ELEVATOR_GEAR_RATIO = 1/50;
    public static final double RADIUS_ELEVATOR_M = 1;
    public static final double ELEVATOR_HEIGHT_M = 1;
    public static final double ELEVATOR_MM_PER_PULSE = 586.0 / 477359; //const for 1 elevator mm/pulse
//119961
//
    //DRIVE
    public static final double DRIVE_GEAR_RATIO = 1/6;
    public static final double DRIVE_WHEEL_CIRCUMEFERENCE_M = Math.PI * 0.1524;

    //GRIPPPER
    public static final double GRIPPER_1_PER_PULSE = 1/0.096923828125;

    //LIMELIGHT //2PI * 0.1524
    public static final double REFLECTIVE_HIGH_HEIGHT_M = 1.12;
    public static final double REFLECTIVE_LOW_HEIGHT_M = 0.6;
    public static final double LIMELIGHT_HEIGHT_M = 1;
    public static final double LIMELIGHT_ANGLE = 1;

    //GAME PIECES
    public static final double CONE_HEIGHT_M = 0.33;
    public static final double CUBE_HEIGHT_M = 0.24;
    
}
