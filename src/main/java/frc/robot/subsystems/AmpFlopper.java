// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorIds;

public class AmpFlopper extends SubsystemBase {
  private CANSparkMax soundFlapMotor;
  private double currentPercentage;
  

  /** Creates a new AmpFlopper. */
  public AmpFlopper() {
    super();
    this.soundFlapMotor = new CANSparkMax(MotorIds.kAmpFlopperMotorCanId, MotorType.kBrushed);

    soundFlapMotor.restoreFactoryDefaults();
    currentPercentage = 0;
    soundFlapMotor.setInverted(true);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    super.periodic();
    SmartDashboard.putNumber("ArmFlopper Percentage", currentPercentage);
    
  }

  public void setMotor(double speed) {
    currentPercentage = speed;
    this.soundFlapMotor.set(speed);
  }




  public void stopMotor() {
    setMotor(0);
  }

  public Command ampFlopperUpCommand() {
    return Commands.startEnd(() -> setMotor(.5), () -> stopMotor(), this);
  }

  public Command ampFlopperDownCommand(double speed) {
    return Commands.startEnd(() -> setMotor(speed), () -> stopMotor(), this);
  }

  public Command ampFlopperUpCommand(double speed) {
    return Commands.startEnd(() -> setMotor(speed), () -> stopMotor(), this);
  }



  public Command ampFlopperDownCommand() {
    return Commands.startEnd(() -> setMotor(-.5), () -> stopMotor(), this);
  }
}
