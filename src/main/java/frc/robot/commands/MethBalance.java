// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSystem;
import frc.robot.subsystems.PLEASE_GOD_HELPME_END_MY_SUFFERING;

public class MethBalance extends CommandBase {

  PLEASE_GOD_HELPME_END_MY_SUFFERING methSystem;
  DriveSystem driveSystem;
  
  double[] y;

  double[] y1;

  double[] ypred;

  int count;
  private double speed;

  public MethBalance(PLEASE_GOD_HELPME_END_MY_SUFFERING methSystem, DriveSystem driveSystem) {
    this.driveSystem = driveSystem;
    this.methSystem = methSystem;
    addRequirements(methSystem, driveSystem);
  }
  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    count = 0;
    y = new double[1];
    y[0] = methSystem.getPitch();
    speed = methSystem.kalmanEstametion(ypred, count);
    ypred = methSystem.predict(y,ypred,count);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    count++;
    for (int i = 0; i < ypred.length ; i++) {      
      if(methSystem.getPitch() > 0){
        if(methSystem.kalmanEstametion(ypred, i) > 0.25) {
          speed = 0.25;
        }
        driveSystem.drive(speed, 0, 0);
      }else if(methSystem.getPitch() < 0){
        if(methSystem.kalmanEstametion(ypred, i) < -0.25) {
          speed = -0.25;
        }
        driveSystem.drive(0, methSystem.kalmanEstametion(ypred, i), 0);
      }
    }
    if(count % 40 == 0){
      y1 = new double[count/40];
      for (int i = 0; i < y.length; i++) {
        y1[i] = y[i];
      }
      y = new double[(count/40)+1];
      y[count/40] = methSystem.getPitch();
  
      ypred = methSystem.predict(y, ypred, 0);
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
    return methSystem.measurmant(count) > 11 || methSystem.getPitch() < -11;
  }
}
