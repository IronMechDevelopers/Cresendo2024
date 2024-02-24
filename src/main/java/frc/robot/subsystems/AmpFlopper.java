// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.MotorIds;

public class AmpFlopper extends SubsystemBase {
  private CANSparkMax ampFlopperMotor;

  /** Creates a new AmpFlopper. */
  public AmpFlopper() {
    super();
    this.ampFlopperMotor = new CANSparkMax(MotorIds.kAmpFlopperMotorCanId, MotorType.kBrushed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setMotor(double speed) {
    this.ampFlopperMotor.set(speed);
  }

  public void stopMotor() {
    setMotor(0);
  }

  public Command ampFlopperUpCommand() {
    return Commands.startEnd(() -> setMotor(.25), () -> stopMotor(), this);
  }

  public Command ampFlopperDownCommand() {
    return Commands.startEnd(() -> setMotor(-.25), () -> stopMotor(), this);
  }
}
