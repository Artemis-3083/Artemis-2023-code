package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
    WPI_TalonFX motor;
    static final int speed = 1;
    public Elevator(){
        motor = new WPI_TalonFX(0);
    }
    public void elevatorUp(){
        motor.set(speed);
    }
    public void elevatorDown(){
        motor.set(-speed);
    }
    public void stop(){
        motor.set(0);
    }
    public double getHeight(){
        return((motor.getSelectedSensorPosition()
                /Constants.TALON_FX_PPR)*Constants.GEAR_RATIO_ELEVATOR
                *(Math.PI*Constants.RADIUS_ELEVATOR*2));
    }
}
