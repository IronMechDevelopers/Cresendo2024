package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class ZeroGyro extends Command {
private DriveSubsystem drive;

public ZeroGyro(DriveSubsystem driveSubsystem){
    drive=driveSubsystem;
}


     // What we need to set up the command.
  @Override
  public void initialize() {
  }

  // Runs the commands given and it runs every .2 seconds.
  @Override
  public void execute() {
    drive.zeroHeading();
  }

  // What it needs to do after the command is done.
  @Override
  public void end(boolean interrupted) {
    
  }

  // How it checks the command is done.
  @Override
  public boolean isFinished() {
    return true;
  }
    
}
