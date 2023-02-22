package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.sim.ElevatorSystemSim;

public class ElevatorSystem extends SubsystemBase {
    CANSparkMax  motor;
    DigitalInput digitalInput;
    static final int speed = 1;
    RelativeEncoder encoder;
    private final ElevatorSystemSim sim;

    public ElevatorSystem(){
        digitalInput = new DigitalInput(0);
        motor = new CANSparkMax(0,MotorType.kBrushless);
        encoder = motor.getEncoder();

        if (Robot.isSimulation()) {
            sim = new ElevatorSystemSim();
        } else {
            sim = null;
        }
    }

    public void elevatorUp(){
        motor.set(speed);
    }

    public void elevatorDown(){
        motor.set(-speed);
    }

    public void elevatorSetSpeed(double speed){
        motor.set(speed);
    }

    public void stop(){
        motor.set(0);
    }

    public void resetEncoder(){
        if (Robot.isSimulation()) {
            sim.reset();
        } else {
            encoder.setPosition(0);
        }
    }

    public boolean getLimitSwitch(){
        if (Robot.isSimulation()) {
            return sim.getLimitSwitch();
        } else {
            return digitalInput.get();
        }
    }

    public double getHeight(){
        if (Robot.isSimulation()) {
            return sim.getHeightMeters();
        } else {
            return(encoder.getPosition()
                    *Constants.GEAR_RATIO_ELEVATOR
                    *(Math.PI*Constants.RADIUS_ELEVATOR*2));
        }
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Elevator.Height", getHeight());
        SmartDashboard.putBoolean("Elevator.Limit", getLimitSwitch());
    }

    @Override
    public void simulationPeriodic() {
        sim.update(motor.get(), 0.02);
    }
}
