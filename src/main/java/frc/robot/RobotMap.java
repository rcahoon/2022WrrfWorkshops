package frc.robot;

/**
 * This class defines which ports and CAN Bus IDs are used for all of the
 * devices controlled by the program.
 *
 * Having a centralized place to define these makes it easy to check that
 * multiple devices haven't been accidentally assigned the same ports/IDs.
 */
public class RobotMap {
  // CAN Bus Device IDs
  public static class CanBusIds {
    public static final int DRIVE_LEFT_MOTOR = 1;
    public static final int DRIVE_RIGHT_MOTOR = 2;
    public static final int ARM_MOTOR = 23;
  }

  // PWM Ports
  public static class PwmPorts {
  }

  // Analog Input Ports
  public static class AnalogPorts {
  }

  // Solenoids
  public static class SolenoidPorts {
    public static final int GRIPPER = 1;
  }
}
