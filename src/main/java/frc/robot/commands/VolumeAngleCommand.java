// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.VolumeSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class VolumeAngleCommand extends PIDCommand {
  /** Creates a new SoundAngleCommand. */
  public VolumeAngleCommand(VolumeSubsystem volume, double goalAngle) {
    super(
        // The controller that the command will use
        new PIDController(.025, 0, 0),
        // This should return the measurement
        () -> volume.getAngle(),
        // This should return the setpoint (can also be a constant)
        () -> goalAngle,
        // This uses the output
        output -> {
          volume.setGoalAngle(goalAngle);
          volume.setMotor(output);
        });
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    addRequirements(volume);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
