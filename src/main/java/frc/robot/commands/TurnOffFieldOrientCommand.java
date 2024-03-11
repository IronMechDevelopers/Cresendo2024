// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class TurnOffFieldOrientCommand extends Command {
  private DriveSubsystem m_robotDrive;

  /** Creates a new TurnOffFieldOrientCommand. */
  public TurnOffFieldOrientCommand(DriveSubsystem m_robotDrive) {

    this.m_robotDrive = m_robotDrive;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    m_robotDrive.setFieldOrientation(false);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    m_robotDrive.setFieldOrientation(true);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
