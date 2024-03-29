package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.StagingSubsystem;

public class TestingSubsystemsCommand extends SequentialCommandGroup {
        public TestingSubsystemsCommand(DriveSubsystem driveSubsystem, StagingSubsystem stagingSubsytem,
                        ShooterSubsystem shooterSubsystem) {
                addCommands(
                                driveSubsystem.driveCommand(.5, 0, 0).withTimeout(5),
                                new WaitCommand(.5),
                                driveSubsystem.driveCommand(.5, .5, 0).withTimeout(5),
                                new WaitCommand(.5),
                                shooterSubsystem.setMotorToPercentCommand(.50).withTimeout(5),
                                new WaitCommand(.5),
                                stagingSubsytem.drivingIntakeCommand().withTimeout(15));
        }

}