package frc.robot.commands;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootWarmpUpCommand extends Command {
    private ShooterSubsystem m_shooterSubsystem;
    private double rpm;

    public ShootWarmpUpCommand(ShooterSubsystem shooterSubsystem, double rpm) {
        super();
        this.m_shooterSubsystem = shooterSubsystem;
        this.rpm = rpm;

        addRequirements(m_shooterSubsystem);
    }

    @Override
    public void initialize() {
    }

    // Runs the commands given and it runs every .2 seconds.
    @Override
    public void execute() {
        m_shooterSubsystem.setMotorToRPM(rpm);

    }

    // What it needs to do after the command is done.
    @Override
    public void end(boolean interrupted) {
        m_shooterSubsystem.stopMotor();
        DataLogManager.log("ENDING SHOOT");

    }

    // How it checks the command is done.
    @Override
    public boolean isFinished() {
        return false;
    }
}
