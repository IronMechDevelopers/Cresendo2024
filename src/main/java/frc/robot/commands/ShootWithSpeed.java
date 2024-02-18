package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootWithSpeed extends Command {
    private ShooterSubsystem m_shooterSubsystem;
    private String speedType;

    public ShootWithSpeed(ShooterSubsystem shooterSubsystem, String speedType) {
        super();
        this.m_shooterSubsystem = shooterSubsystem;
        this.speedType = speedType;

        addRequirements(m_shooterSubsystem);

    }

    @Override
    public void initialize() {
    }

    // Runs the commands given and it runs every .2 seconds.
    @Override
    public void execute() {
        double percent = SmartDashboard.getNumber(speedType, .25);
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
