package frc.robot.commands;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.StagingSubsytem;

public class IntakeCommand extends Command {
    private StagingSubsytem m_stagingSubsytem;
    private double speed;



    public IntakeCommand(StagingSubsytem stagingSubsytem, double speed) {
        super();
        this.m_stagingSubsytem = stagingSubsytem;
        this.speed = speed;

        addRequirements(stagingSubsytem);
    }

    @Override
    public void initialize() {
    }

    // Runs the commands given and it runs every .2 seconds.
    @Override
    public void execute() {
        m_stagingSubsytem.setMotor(speed);

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
