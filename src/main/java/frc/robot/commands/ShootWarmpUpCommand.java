package frc.robot.commands;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootWarmpUpCommand extends Command {
    private ShooterSubsystem m_shooterSubsystem;
    private double percent;


    public ShootWarmpUpCommand(ShooterSubsystem shooterSubsystem) {
        super();
        this.m_shooterSubsystem = shooterSubsystem;
        this.percent = percent;

        addRequirements(m_shooterSubsystem);


    }

    @Override
    public void initialize() {
    }

    // Runs the commands given and it runs every .2 seconds.
    @Override
    public void execute() {
        m_shooterSubsystem.setMotorToPercent(percent);
        // m_shooterSubsystem.setMotorToRPM(4500.0);
    }

    // What it needs to do after the command is done.
    @Override
    public void end(boolean interrupted) {
        m_shooterSubsystem.stopMotor();

    }

    // How it checks the command is done.
    @Override
    public boolean isFinished() {
        return false;
    }
}
