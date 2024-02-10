package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.StagingSubsytem;

public class DrivingIntake extends SequentialCommandGroup {

    public DrivingIntake(StagingSubsytem stagingSubsytem) {
        addCommands(new IntakeCommand(stagingSubsytem, Constants.SpeedConstants.InTakeSpeed), new IntakeCommand(stagingSubsytem,Constants.SpeedConstants.OuttakeSpeed).withTimeout(2));
    }

}
