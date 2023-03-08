// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.subsystems.DriveSystem;
// import frc.robot.subsystems.VisionSystem;

// public class DriveUntilDistanceFromTag extends CommandBase {

//   DriveSystem driveSystem;
//   VisionSystem visionSystem;
//   double goal; //in m
//   double distance; //in m
  
//   public DriveUntilDistanceFromTag(double goal, DriveSystem driveSystem, VisionSystem visionSystem) {
//     this.goal = goal;
//     this.driveSystem = driveSystem;
//     this.visionSystem = visionSystem;
//     addRequirements(visionSystem, driveSystem);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {}

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     distance = visionSystem.getTagDistance();
//     if(distance > goal){
//       driveSystem.drive(0.2*distance, 0, 0);
//     }else{
//       driveSystem.drive(0, distance/goal, 0);
//     }
//   }
    

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     driveSystem.stop();
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return distance < goal+0.01 && distance > goal-0.01;
//   }
// }
