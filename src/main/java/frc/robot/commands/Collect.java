package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class Collect extends Command {
    private ShooterSubsystem mShooterSubsystem;
    private double mSpeed;

    public Collect(ShooterSubsystem shooterSubsystem, double speed){ 
        this.mShooterSubsystem = shooterSubsystem;
        this.mSpeed = speed;
    }

    // What we need to set up the command.
    @Override
    public void initialize() {
    }

    // Runs the commands given and it runs every .2 seconds.
    @Override
    public void execute() {
        mShooterSubsystem.collectorPower(mSpeed);
    }

    // What it needs to do after the command is done.
    @Override
    public void end(boolean interrupted) {
        mShooterSubsystem.collectorPower(0);
    }

    // How it checks the command is done.
    @Override
    public boolean isFinished() {
        return false;
    }

}
