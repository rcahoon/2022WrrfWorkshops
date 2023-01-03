package frc.robot.commands;

import frc.robot.subsystems.Drive;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * Example showing a Command class implemented "from scratch". For most Commands, you can generally
 * use the pre-defined "convenience" Command classes to achieve the same thing with much less code.
 * See https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html for
 * general information, and {@link DriveCommands.driveForwardForSeconds} for the equivalent code to
 * this class.
 */
public class DriveForward extends CommandBase {
  private final Drive m_drive;
  private final double m_duration;

  private final Timer m_timer = new Timer();

  /**
   * Creates a new DriveForward Command.
   *
   * @param drive The drive subsystem used by this command.
   * @param seconds How long (in seconds) to drive forward for.
   */
  public DriveForward(Drive drive, double seconds) {
    m_drive = drive;
    m_duration = seconds;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drive.setForwardAndSteer(0.5, 0);
    
    m_timer.reset();
    m_timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_timer.hasElapsed(m_duration);
  }
}
