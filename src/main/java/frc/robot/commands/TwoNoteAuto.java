package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.StagingSubsytem;

public class TwoNoteAuto extends SequentialCommandGroup {

        public TwoNoteAuto(StagingSubsytem stagingSubsytem, ShooterSubsystem shooterSubsystem,
                        DriveSubsystem driveSubsystem) {
                addCommands(
                                new ParallelRaceGroup(
                                                shooterSubsystem.setMotorToPercentCommand(
                                                                "Fast Speed"),
                                                new WaitCommand(.25)
                                                                .andThen(stagingSubsytem.runIntakeCommand())
                                                                .withTimeout(.75)),
                                new ParallelRaceGroup(driveSubsystem.driveCommand(-.25, 0, 0).withTimeout(1.5),
                                                stagingSubsytem.drivingIntakeCommand()),
                                new ParallelRaceGroup(driveSubsystem.driveCommand(.25, 0, 0).withTimeout(1.75),
                                                shooterSubsystem.setMotorToPercentCommand(
                                                                "Fast Speed")),
                                new ParallelRaceGroup(
                                                shooterSubsystem.setMotorToPercentCommand(
                                                                "Fast Speed"),
                                                new WaitCommand(.25)
                                                                .andThen(stagingSubsytem.runIntakeCommand()
                                                                                .withTimeout(.75))));
        }

}
