package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
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
    
    TalonFX  motor;
    DigitalInput digitalInput;
    static final int speed = 1;
    private final ElevatorSystemSim sim;

    public ElevatorSystem(){
        digitalInput = new DigitalInput(0);
        motor = new TalonFX(5);

        if (Robot.isSimulation()) {
            sim = new ElevatorSystemSim();
        } else {
            sim = null;
        }
    }

    public void elevatorUp(){
        motor.set(ControlMode.PercentOutput, speed);
    }

    public void elevatorDown(){
        motor.set(ControlMode.PercentOutput, -speed);
    }

    public void elevatorSetSpeed(double speed){
        motor.set(ControlMode.PercentOutput, speed);
    }

    public void stop(){
        motor.set(ControlMode.PercentOutput, 0);
    }

    public void resetEncoder(){
        if (Robot.isSimulation()) {
            sim.reset();
        } else {
            motor.setSelectedSensorPosition(0);
        }
    }

    public boolean getLimitSwitch(){
        if (Robot.isSimulation()) {
            return sim.getLimitSwitch();
        } else {
            return !digitalInput.get();
        }
    }

    public double getHeight(){
        if (Robot.isSimulation()) {
            return sim.getHeightMeters();
        } else {
            return motor.getSelectedSensorPosition() * Constants.ELEVATOR_MM_PER_PULSE;
            // (encoder.getPosition()
                    // *Constants.GEAR_RATIO_ELEVATOR
                    // *(Math.PI*Constants.RADIUS_ELEVATOR_M*2));
        }
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Elevator.Height", getHeight());
        SmartDashboard.putBoolean("Elevator.Limit", getLimitSwitch());
    }

    /*@Override
    public void simulationPeriodic() {
        sim.update(motor.get(), 0.02);
    }*/
}
