package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Gripper;
import static edu.wpi.first.wpilibj2.command.CommandGroupBase.sequence;

/**
 * Put command primitives that operate the Gripper subsystem in this class.
 */
public class GripperCommands {
  /** The amount of time it takes the gripper to open (in seconds) */
  private static final double TIME_TO_OPEN = 1;
  /** The amount of time it takes the gripper to close (in seconds) */
  private static final double TIME_TO_CLOSE = 1;

  private final Gripper m_gripper;

  public GripperCommands(Gripper gripper) {
    m_gripper = gripper;
  }

  /**
   * Produce a Command object which will open the gripper.
   * It will wait an amount of time for the gripper to finish moving.
   *
   * @return The generated Command
   */
  public Command openGripper() {
    return sequence(
        new InstantCommand(() -> m_gripper.openGripper()),
        new WaitCommand(TIME_TO_OPEN));
  }

  /**
   * Produce a Command object which will close the gripper.
   * It will wait an amount of time for the gripper to finish moving.
   *
   * @return The generated Command
   */
  public Command closeGripper() {
    return sequence(
        new InstantCommand(() -> m_gripper.closeGripper()),
        new WaitCommand(TIME_TO_CLOSE));
  }
}
