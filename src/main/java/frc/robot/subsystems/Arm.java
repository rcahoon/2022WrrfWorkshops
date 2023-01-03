package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.RobotMap;

/**
 * Subsystem that represents the single-joint arm.
 */
public class Arm extends PIDSubsystem {
    /** Angle (in degrees) of the arm's raised position */
    public static final double RAISED_ANGLE = 55;
    /** Angle (in degrees) of the arm's lowered position */
    public static final double LOWERED_ANGLE = -40;

    /** Scale factor converting encoder units to degrees */
    private static final double ENCODER_TO_ANGLE = 0.02779;
    /** Encoder reading (in encoder units) which represents an angle of 0 */
    private static final double ENCODER_ZERO = 3550;

    /**
     * Feed-forward gain for the controller
     * This will apply motor power to counteract the force of gravity, as
     * predicted by the angle of the arm.
     */
    private static double ff_gain = 0.075;
    /**
     * Proportional gain for the PID controller
     * This controls the strength of the controller's response to error in the
     * arm's position: it will apply more motor power when the arm is further
     * away from the target angle.
     */
    private static final double P_GAIN = 0.0085;
    /**
     * Integral gain for the PID controller
     * This controls the strength of the controller's response to steady-state
     * error in the arm's position: it will apply motor power to counteract
     * persistent forces like gravity. Unlike the feed-forward controller, the
     * integral controller can automatically respond to changes in the force of
     * gravity (such as when the gripper picks up a game piece, increasing the
     * total weight carried by the arm), but increases the risk of making the
     * controller unstable.
     */
    private static final double I_GAIN = 0.003;
    /**
     * Derivative gain for the PID controller
     * This controls the strength of the controller's response to change in
     * the arm's position: it will apply motor power to try to slow down the
     * arm, in order to counteract inertia.
     */
    private static final double D_GAIN = 0.002;

    private WPI_TalonSRX m_motor = new WPI_TalonSRX(RobotMap.CanBusIds.ARM_MOTOR);

    public Arm() {
        super(new PIDController(P_GAIN, I_GAIN, D_GAIN));
        getController().setTolerance(5);
        getController().setSetpoint(LOWERED_ANGLE);

        m_motor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
        m_motor.setNeutralMode(NeutralMode.Brake);
        m_motor.configNeutralDeadband(0.001);

        addChild("Motor", m_motor);

        // Enable the PID controller
        enable();
    }

    /**
     * Returns the current angle of the arm
     *
     * @return The angle of the arm, in degrees
     */
    public double getAngle() {
        return (m_motor.getSelectedSensorPosition() - ENCODER_ZERO) * ENCODER_TO_ANGLE;
    }

    /**
     * Uses the output from the PIDController.
     *
     * @param output the output of the PIDController
     * @param setpoint the setpoint of the PIDController (for feedforward)
     */
    @Override
    protected void useOutput(double output, double setpoint) {
        m_motor.set(output + ff_gain * Math.cos(Math.toRadians(setpoint)));
    }

    /**
     * Supplies the arm angle to the PID controller
     */
    @Override
    protected double getMeasurement() {
        return getAngle();
    }

    /**
     * Setup extra Smart Dashboard / Shuffleboard controls
     */
    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);

        builder.addDoubleProperty("FF", () -> ff_gain, (double val) -> { ff_gain = val; });
        builder.addDoubleProperty("Angle", this::getAngle, (double val) -> {});
        builder.addDoubleProperty("Raw Measurement", m_motor::getSelectedSensorPosition, (double val) -> {});
    }

    @Override
    public void periodic() {
        // Run the PID controller if not in Test mode
        // (in Test mode, we can manually control the motor directly)
        if (DriverStation.isEnabled() && !DriverStation.isTest()) {
            super.periodic();
        } else {
            m_motor.set(0);
            getController().reset();
        }
    }
}
