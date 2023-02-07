package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSystem extends SubsystemBase {
    CANSparkMax  motor;
    DigitalInput digitalInput;
    Encoder encoder;
    static final int speed = 1;
    public ElevatorSystem(){
        digitalInput = new DigitalInput(0);
        motor = new CANSparkMax(0,MotorType.kBrushless);
        encoder = new Encoder(0, 0);
        
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
    public void resetEncoder(){
        encoder.reset();
    }
    public boolean getLimitSwitch(){
        return digitalInput.get();
    }
    public double getHeight(){
        return(encoder.get()
                *Constants.GEAR_RATIO_ELEVATOR
                *(Math.PI*Constants.RADIUS_ELEVATOR*2));
    }
}
