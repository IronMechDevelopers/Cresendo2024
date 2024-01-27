package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class Staging extends Command {
    private ShooterSubsystem mStagingSubsystem;
    private double mSpeed;
    public Staging(ShooterSubsystem stagingSubsystem, double speed){
        this.mStagingSubsystem = stagingSubsystem;
        this.mSpeed = speed;
    }
     // What we need to set up the command.
  @Override
  public void initialize() {
  }

  // Runs the commands given and it runs every .2 seconds.
  @Override
  public void execute() {
    mStagingSubsystem.stagingPower(mSpeed);
  }

  // What it needs to do after the command is done.
  @Override
  public void end(boolean interrupted) {
    mStagingSubsystem.stagingPower(0);
  }

  // How it checks the command is done.
  @Override
  public boolean isFinished() {
    return false;
  }
    
}
