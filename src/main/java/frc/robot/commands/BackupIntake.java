package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.StagingSubsytem;

public class BackupIntake extends Command {
    private StagingSubsytem m_stagingSubsytem;

    public BackupIntake(StagingSubsytem stagingSubsytem) {
        super();
        this.m_stagingSubsytem = stagingSubsytem;

        addRequirements(stagingSubsytem);
    }

    @Override
    public void initialize() {
    }

    // Runs the commands given and it runs every .2 seconds.
    @Override
    public void execute() {
        m_stagingSubsytem.setMotor(-.05);

    }

    // What it needs to do after the command is done.
    @Override
    public void end(boolean interrupted) {
        m_stagingSubsytem.stopMotor();

    }

    // How it checks the command is done.
    @Override
    public boolean isFinished() {
        return false;
    }

}
