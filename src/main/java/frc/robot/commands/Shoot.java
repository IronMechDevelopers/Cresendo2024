package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.StagingSubsytem;

public class Shoot extends SequentialCommandGroup {
    public Shoot(StagingSubsytem stagingSubsytem, ShooterSubsystem shooterSubsystem) {
        addCommands(
                new ParallelRaceGroup(
                        new ShootWarmpUpCommand(shooterSubsystem),
                        new WaitCommand(.50)
                                .andThen(new IntakeCommand(stagingSubsytem, Constants.SpeedConstants.InTakeSpeed))
                                .withTimeout(5)));
    }

}