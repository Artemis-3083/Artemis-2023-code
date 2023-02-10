package frc.robot.sim;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.Constants;

public class ElevatorSystemSim {

    // those definitions are robot-specific
    // they are based on actual robot structure, sizes and such
    // please update them according to your actual robot
    public static final double MOTOR_TO_WHEEL_GEAR_RATIO = Constants.GEAR_RATIO_ELEVATOR;
    public static final double DRUM_RADIUS_M = Units.inchesToMeters(2.5);
    public static final double CARRIAGE_MASS_KG = 0.4;
    public static final double MIN_HEIGHT_METERS = 0;
    public static final double MAX_HEIGHT_METERS = Constants.ELEVATOR_HEIGHT;
    public static final int MOTOR_COUNT = 1;
    public static final DCMotor MOTORS_TYPE = DCMotor.getCIM(MOTOR_COUNT);
    public static final boolean SIMULATE_GRAVITY = true;
    private static final double HEIGHT_LIMIT_SWITCH_ACTIVE = 0.05;

    private final ElevatorSim sim;
    private double resetPos;

    public ElevatorSystemSim() {
        sim = new ElevatorSim(
                MOTORS_TYPE,
                MOTOR_TO_WHEEL_GEAR_RATIO,
                CARRIAGE_MASS_KG,
                DRUM_RADIUS_M,
                MIN_HEIGHT_METERS,
                MAX_HEIGHT_METERS,
                SIMULATE_GRAVITY
        );

        resetPos = 0;
    }

    public void reset() {
        resetPos = sim.getPositionMeters();
    }

    public double getHeightMeters() {
        return sim.getPositionMeters() - resetPos;
    }

    public boolean getLimitSwitch() {
        return getHeightMeters() <= HEIGHT_LIMIT_SWITCH_ACTIVE;
    }

    public void update(double speed, double dt) {
        sim.setInput(speed * RobotController.getBatteryVoltage());
        sim.update(dt);
    }
}
