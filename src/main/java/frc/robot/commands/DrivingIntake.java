package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.StagingSubsytem;

public class DrivingIntake extends SequentialCommandGroup {

    public DrivingIntake(StagingSubsytem stagingSubsytem) {
        addCommands(new IntakeCommand(stagingSubsytem), new BackupIntake(stagingSubsytem).withTimeout(2));
    }

}
