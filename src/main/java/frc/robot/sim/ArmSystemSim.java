package frc.robot.sim;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class ArmSystemSim {

    // those definitions are robot-specific
    // they are based on actual robot structure, sizes and such
    // please update them according to your actual robot
    public static final double FIRST_MOTOR_TO_ARM_GEAR_RATIO = 4;
    public static final double FIRST_ARM_MASS_KG = 10;
    public static final double FIRST_ARM_LENGTH_METERS = 0.3;
    public static final int FIRST_MOTOR_COUNT = 2;
    public static final DCMotor FIRST_MOTORS_TYPE = DCMotor.getCIM(FIRST_MOTOR_COUNT);
    public static final double FIRST_MIN_ANGLE_RAD = 0;
    public static final double FIRST_MAX_ANGLE_RAD = 90;
    public static final double FIRST_MOMENT_OF_INERTIA = (Math.pow(FIRST_ARM_MASS_KG * FIRST_ARM_LENGTH_METERS, 2)) / 3;

    public static final double SECOND_MOTOR_TO_ARM_GEAR_RATIO = 4;
    public static final double SECOND_ARM_MASS_KG = 10;
    public static final double SECOND_ARM_LENGTH_METERS = 0.3;
    public static final int SECOND_MOTOR_COUNT = 2;
    public static final DCMotor SECOND_MOTORS_TYPE = DCMotor.getCIM(SECOND_MOTOR_COUNT);
    public static final double SECOND_MIN_ANGLE_RAD = 0;
    public static final double SECOND_MAX_ANGLE_RAD = 90;
    public static final double SECOND_MOMENT_OF_INERTIA = (Math.pow(SECOND_ARM_MASS_KG * SECOND_ARM_LENGTH_METERS, 2)) / 3;

    public static final boolean SIMULATE_GRAVITY = false;

    private final SingleJointedArmSim mFirstArm;
    private final SingleJointedArmSim mSecondArm;

    public ArmSystemSim() {
        mFirstArm = new SingleJointedArmSim(
                FIRST_MOTORS_TYPE,
                FIRST_MOTOR_TO_ARM_GEAR_RATIO,
                FIRST_MOMENT_OF_INERTIA,
                FIRST_ARM_LENGTH_METERS,
                FIRST_MIN_ANGLE_RAD,
                FIRST_MAX_ANGLE_RAD,
                FIRST_ARM_MASS_KG,
                SIMULATE_GRAVITY
        );

        mSecondArm = new SingleJointedArmSim(
                SECOND_MOTORS_TYPE,
                SECOND_MOTOR_TO_ARM_GEAR_RATIO,
                SECOND_MOMENT_OF_INERTIA,
                SECOND_ARM_LENGTH_METERS,
                SECOND_MIN_ANGLE_RAD,
                SECOND_MAX_ANGLE_RAD,
                SECOND_ARM_MASS_KG,
                SIMULATE_GRAVITY
        );
    }

    public double getFirstJointAngleDegrees() {
        return Units.radiansToDegrees(mFirstArm.getAngleRads());
    }

    public double getSecondJointAngleDegrees() {
        return Units.radiansToDegrees(mSecondArm.getAngleRads());
    }

    public void update(double firstSpeed, double secondSpeed, double dt) {
        mFirstArm.setInput(firstSpeed * RobotController.getBatteryVoltage());
        mFirstArm.update(dt);

        mSecondArm.setInput(secondSpeed * RobotController.getBatteryVoltage());
        mSecondArm.update(dt);
    }
}
