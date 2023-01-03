// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commandgroups.ArmCommands;
import frc.robot.commandgroups.DriveCommands;
import frc.robot.commandgroups.GripperCommands;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Gripper;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import static edu.wpi.first.wpilibj2.command.CommandGroupBase.parallel;
import static edu.wpi.first.wpilibj2.command.CommandGroupBase.sequence;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Drive m_drive = new Drive();
  private final DriveCommands m_driveCommands = new DriveCommands(m_drive);

  private final Arm m_arm = new Arm();
  private final ArmCommands m_armCommands = new ArmCommands(m_arm);

  private final Gripper m_gripper = new Gripper();
  private final GripperCommands m_gripperCommands = new GripperCommands(m_gripper);

  private final Joystick m_joystick = new Joystick(1);

  private Command m_autoCommand = sequence(
      m_gripperCommands.closeGripper(),
      parallel(
        m_driveCommands.driveForwardForSeconds(2),
        m_armCommands.raiseArm()),
      m_driveCommands.driveForwardForSeconds(1),
      m_gripperCommands.openGripper(),
      m_armCommands.lowerArm(),
      m_driveCommands.driveBackwardForSeconds(3));

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    configureJoystickBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureJoystickBindings() {
    // Drive joystick bindings

    m_drive.setDefaultCommand(
        new RunCommand(() -> m_drive.setForwardAndSteer(
              Constants.DRIVE_FORWARD_SCALE * (-1) * m_joystick.getY(),
              Constants.DRIVE_STEER_SCALE * m_joystick.getX()),
                      m_drive));

    // Arm joystick bindings
                      
    m_arm.setDefaultCommand(
      new RunCommand(() -> {
        if (Math.abs(m_joystick.getRawAxis(3)) > Constants.ARM_JOYSTICK_DEADBAND) {
          m_arm.setSetpoint(m_arm.getAngle() + Constants.ARM_JOYSTICK_SPEED * (-1) * m_joystick.getRawAxis(3));
        }
      },
      m_arm));
      
    new JoystickButton(m_joystick, 1).whenPressed(m_gripperCommands.closeGripper());
    new JoystickButton(m_joystick, 2).whenPressed(m_gripperCommands.openGripper());
    new JoystickButton(m_joystick, 3).whenPressed(m_armCommands.raiseArm());
    new JoystickButton(m_joystick, 4).whenPressed(m_armCommands.lowerArm());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_autoCommand;
  }
}
