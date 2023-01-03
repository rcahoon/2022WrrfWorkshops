package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

/**
 * Subsystem that represents the robot's gripper claw.
 */
public class Gripper extends SubsystemBase {
  private final Solenoid m_solenoid = new Solenoid(PneumaticsModuleType.CTREPCM, RobotMap.SolenoidPorts.GRIPPER);

  public Gripper() {
    addChild("Solenoid", m_solenoid);
  }

  /**
   * Set the gripper pneumatic to the closed position.
   */
  public void closeGripper() {
    m_solenoid.set(true);
  }

  /**
   * Set the gripper pneumatic to the open position.
   */
  public void openGripper() {
    m_solenoid.set(false);
  }
}
