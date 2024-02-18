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
                                getDriveBackwardCommand().withTimeout(5),
                                getDriveSidewaysCommand().withTimeout(5),
                                new ShootWarmpUpCommand(shooterSubsystem).withTimeout(5),
                                new DrivingIntake(stagingSubsytem).withTimeout(15));
        }

        public Command getDriveBackwardCommand() {
                return Commands.startEnd(() -> driveSubsystem.drive(-.25, 0, 0, false),
                                () -> driveSubsystem.drive(0, 0, 0, false), driveSubsystem);
        }

        public Command getDriveSidewaysCommand() {
                return Commands.startEnd(() -> driveSubsystem.drive(0, .25, 0, false),
                                () -> driveSubsystem.drive(0, 0, 0, false), driveSubsystem);
        }

}