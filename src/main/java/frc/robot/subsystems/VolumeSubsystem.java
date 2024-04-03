// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorIds;
import frc.robot.Constants.VolumeConstants;

public class VolumeSubsystem extends SubsystemBase {

  private CANSparkMax soundAngleMotor;
  private DutyCycleEncoder encoder;
  private double currentPercentage;
  private double goalAngle;

  /** Creates a new Volume. */
  public VolumeSubsystem() {
    super();
    goalAngle = VolumeConstants.kVolumeDownAngle;
    currentPercentage = 0.0;
    this.soundAngleMotor = new CANSparkMax(MotorIds.kSoundAngleMotorCanId, MotorType.kBrushed);
    this.soundAngleMotor.setInverted(false);
    DigitalInput input = new DigitalInput(3);
    encoder = new DutyCycleEncoder(input);
    encoder.setPositionOffset(180.0/360);
    encoder.setDutyCycleRange(1.0 / 1024.0, 1023.0 / 1024.0);
    encoder.setDistancePerRotation(360.0);
  }

  @Override
  public void periodic() {
    super.periodic();
    SmartDashboard.putBoolean("is Encoder Cconnected", encoder.isConnected());
    SmartDashboard.putNumber("Sound Angle Percentage", currentPercentage);
    SmartDashboard.putNumber("Sound Angle", getAngle());
    SmartDashboard.putNumber("Sound Goal Angle", goalAngle);
  }

  public double getAngle() {
    double rawAngle = encoder.getDistance();
    return (rawAngle + 180.0) % 360.0 - 180.0;
  }

  public void setMotor(double speed) {
    currentPercentage = speed;
    this.soundAngleMotor.set(speed);
  }

  public void stopMotor() {
    setMotor(0);
  }

  public Command ampAngleCommand(double speed) {
    return Commands.startEnd(() -> setMotor(speed), () -> stopMotor(), this);
  }

  public Command putVolumeUp() {
    return Commands.startEnd(() -> setMotor(.75), () -> stopMotor(), this);
  }
  public Command putVolumeDown() {
    return Commands.startEnd(() -> setMotor(-.5), () -> stopMotor(), this);
  }
  public Command putVolumeDownAuto() {
    return Commands.startEnd(() -> setMotor(-1.0), () -> stopMotor(), this);
  }

  public void setGoalAngle(double goalAngle) {
    this.goalAngle = goalAngle;
  }

  public Command zeroEncoder() {
    return Commands.runOnce(() -> encoder.reset(), this);
  }


}
