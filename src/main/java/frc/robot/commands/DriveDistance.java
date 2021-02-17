// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveDistance extends CommandBase {
  private final Drivetrain m_drive;
  private final double m_distance;
  private final double m_speed;

  /**
   * Creates a new DriveDistance. This command will drive your your robot for a desired distance at
   * a desired speed.
   *
   * @param speed The speed at which the robot will drive
   * @param inches The number of inches the robot will drive
   * @param drive The drivetrain subsystem on which this command will run
   */
  public DriveDistance(double speed, double inches, Drivetrain drive) {
    m_distance = inches;
    m_speed = speed;
    m_drive = drive;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //m_drive.run(0, 0);
   // m_drive.arcadeDrive(0, 0);
    m_drive.resetEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //m_drive.run(m_speed, m_speed);
    //m_drive.arcadeDrive(m_speed, 0);
    SmartDashboard.putNumber("Speed", m_speed);
    SmartDashboard.putNumber("Left Dist.", m_drive.getLeftDistanceMeter());
    SmartDashboard.putNumber("Left Enc", m_drive.getLeftEncoderCount());
    SmartDashboard.putNumber("Right Dist.", m_drive.getRightDistanceMeter());
    SmartDashboard.putNumber("Right Enc", m_drive.getRightEncoderCount());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //m_drive.run(0, 0);
    //m_drive.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Compare distance travelled from start to desired distance
    if(Math.abs(m_drive.getLeftDistanceMeter()) >= m_distance && Math.abs(m_drive.getRightDistanceMeter()) >= m_distance)
      return true;
    return false;
    //return Math.abs(m_drive.getADistanceInch()) >= m_distance;
  }
}
