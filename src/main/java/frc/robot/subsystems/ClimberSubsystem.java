// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {
  private CANSparkMax climberMotor;

  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
    super();
    // this.climberMotor = new CANSparkMax(MotorIds.kClimberMotorCanId,
    // MotorType.kBrushed);

  }

  public void setMotor(double speed) {
    // this.climberMotor.set(speed);
  }

  public void stopMotor() {
    // setMotor(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public Command climberUpCommand() {
    // return Commands.startEnd(() -> setMotor(ClimberConstants.climberSpeedUp), ()
    // -> stopMotor(), this);
    return null;
  }

  public Command climberDownCommand() {
    // return Commands.startEnd(() -> setMotor(ClimberConstants.climberSpeedDown),
    // () -> stopMotor(), this);
    return null;
  }
}
