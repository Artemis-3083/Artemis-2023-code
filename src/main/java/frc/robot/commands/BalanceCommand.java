// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSystem;

public class BalanceCommand extends CommandBase {

  DriveSystem driveSystem;
  private double speed1=0.6;
  private double speed;
  private boolean isOn = false;

  

  public BalanceCommand(DriveSystem driveSystem) {
    this.driveSystem = driveSystem;
    addRequirements(driveSystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveSystem.setresetcount();
    speed = driveSystem.kalmanEstametion();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {  
    if(driveSystem.getPitch() > 0) {
    speed = Math.abs(driveSystem.kalmanEstametion());
    //takes our speed from kalmanestimation and defines it (absolute value because it's variables can be negative)    
     if (driveSystem.getPitch() > 0) {
      //checks what state the ramp is   
      if(driveSystem.getPitch() >= 8) {         
         if(Math.abs(driveSystem.getPitch())>8){           
           speed = speed1;          
           speed1 = speed-0.0001;
           driveSystem.drive(speed1, 0, 0);
           //driveSystems us     
          }
          else {       
            if (driveSystem.getPitch()<8 && driveSystem.getPitch()>1){
              
            }

            //for when the code is unnsure what to do   
          }        
        }  
        else {
          if(speed > 0.5) {
            speed = 0.5;//maximum speed
          }
        }
        driveSystem.drive(speed, 0, 0);

      } else if (driveSystem.getPitch() < 0) {

        if(driveSystem.getPitch() <= -6) {
          if(Math.abs(driveSystem.getPitch()) > 8){
            speed = speed1;
            speed1 = speed-0.0001;
            driveSystem.drive(0, speed1, 0);
          }else{
            if (driveSystem.getPitch()<-8 && driveSystem.getPitch()> 1){
              driveSystem.drive(speed, 0, 0);
            }
          }
        }
        else {
          if(speed > 0.5) {
            speed = 0.5;
          }
        }
        //this whole command is the same as the last one except it's for the other direction
        driveSystem.drive(0, speed, 0);
      }
    SmartDashboard.putNumber("speed1", speed1);
    SmartDashboard.putNumber("speed", speed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return driveSystem.getPitch() >= -1 && driveSystem.getPitch() <= 1;
  }
}