package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Arm;
import static edu.wpi.first.wpilibj2.command.CommandGroupBase.sequence;

/**
 * Put command primitives that operate the Arm subsystem in this class.
 */
public class ArmCommands {
  private final Arm m_arm;

  public ArmCommands(Arm arm) {
    m_arm = arm;
  }

  /**
   * Produce a Command object which will move the arm to the given angle.
   * When the Command finishes, the arm will be at the target angle.
   * 
   * @param angle The target angle to which the arm should move.
   * @return The generated Command
   */
  public Command moveTo(double angle) {
    return sequence(
        new InstantCommand(() -> m_arm.setSetpoint(angle)),
        new WaitUntilCommand(() -> m_arm.getController().atSetpoint()));
  }

  /**
   * Produce a Command object which will move the arm to the raised position.
   * When the Command finishes, the arm will be at the raised angle.
   * 
   * @return The generated Command
   */
  public Command raiseArm() {
    return moveTo(Arm.RAISED_ANGLE);
  }

  /**
   * Produce a Command object which will move the arm to the lowered position.
   * When the Command finishes, the arm will be at the lowered angle.
   * 
   * @return The generated Command
   */
  public Command lowerArm() {
    return moveTo(Arm.LOWERED_ANGLE);
  }
}
