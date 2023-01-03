// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

/**
 * Subsystem that represents the robot's drive system
 * for moving the robot around the field.
 */
public class Drive extends SubsystemBase {
  private MotorController m_leftMotor = new WPI_TalonFX(RobotMap.CanBusIds.DRIVE_LEFT_MOTOR);
  private MotorController m_rightMotor = new WPI_TalonFX(RobotMap.CanBusIds.DRIVE_RIGHT_MOTOR);

  /**
   * Set the amount of power being sent to the drive motors.
   *
   * @param left The amount of power (-1 ... 1) to send to the left motor(s).
   * @param right The amount of power (-1 ... 1) to send to the right motor(s).
   */
  public void setMotorPowers(double left, double right) {
    // NOTE: Advantage here that you can easily add more motors to the left/right sides of your drive
    m_leftMotor.set(left);
    m_rightMotor.set(right);
  }

  /**
   * Set the amount of power being sent to the drive motors.
   *
   * @param forward The amount of power (-1 ... 1) to direct to moving the robot forward/backward.
   * @param steer The amount of power (-1 ... 1) to direct to turning the robot left/right.
   */
  public void setForwardAndSteer(double forward, double steer) {
    setMotorPowers(forward + steer, forward - steer);
  }

  /**
   * Stop sending power to the drive motors.
   */
  public void stopMotors() {
    setMotorPowers(0, 0);
  }

  @Override
  public void periodic() {
    super.periodic();
    
    // NOTE: This is where you would do odometry to track the position of the
    // robot on the field; e.g. for use in autonomous.
  }
}
