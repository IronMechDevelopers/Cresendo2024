package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.StagingSubsytem;

public class TestingSubsystemsCommand extends SequentialCommandGroup {
        private DriveSubsystem driveSubsystem;

        public TestingSubsystemsCommand(DriveSubsystem driveSubsystem, StagingSubsytem stagingSubsytem,
                        ShooterSubsystem shooterSubsystem) {
                addCommands(
                                driveSubsystem.driveCommand(.25, 0, 0).withTimeout(5),
                                driveSubsystem.driveCommand(0, .25, 0).withTimeout(5),
                                shooterSubsystem.setMotorToPercentCommand(.50).withTimeout(5),
                                stagingSubsytem.drivingIntakeCommand().withTimeout(15));
        }

}