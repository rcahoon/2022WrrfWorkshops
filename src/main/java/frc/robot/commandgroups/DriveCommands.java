package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.subsystems.Drive;

/**
 * Put command primitives that operate the Drive subsystem in this class.
 */
public class DriveCommands {
  /** The amount of power (0 ... 1) to send to the drive motors with the drive commands */
  private static final double DRIVE_POWER = 0.1;

  private final Drive m_drive;

  public DriveCommands(Drive drive) {
    m_drive = drive;
  }

  /**
   * Produce a Command object which will drive the robot forward for the given
   * amount of time and then stop.
   * 
   * @param seconds The amount of time to drive for
   * @return The generated Command
   */
  public Command driveForwardForSeconds(double seconds) {
    return new StartEndCommand(
        () -> m_drive.setForwardAndSteer(DRIVE_POWER, 0),
        () -> m_drive.stopMotors(),
        m_drive)
        .withTimeout(seconds);

    // NOTE: The above has the same functionality as
    // return new DriveForward(m_drive, seconds);
    // See the comments on the DriveForward class
  }

  /**
   * Produce a Command object which will drive the robot forward for the given
   * amount of time and then stop.
   * 
   * @param seconds The amount of time to drive for
   * @return The generated Command
   */
  public Command driveBackwardForSeconds(double seconds) {
    return new StartEndCommand(
        () -> m_drive.setForwardAndSteer(-DRIVE_POWER, 0),
        () -> m_drive.stopMotors(),
        m_drive)
        .withTimeout(seconds);
  }
}
