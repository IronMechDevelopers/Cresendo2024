// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.BlinkinConstants;
import frc.robot.Constants.GlobalConstants;
import frc.robot.subsystems.StagingSubsystem.StagingState;
import edu.wpi.first.wpilibj.Timer;

public class Blinkin extends SubsystemBase {

  private static Blinkin blinkin;
  private Spark blinkinController;

  private double initialTime, currentTime;

  private boolean useReturnToRobotStateTimer;
  private double returnToRobotStateTimeLimit;

  private double lastFlashTime, flashColor, flashRate;
  private boolean useFlashColor, flashOn;

  private double currentColor;
  private StagingSubsystem stagingSubsystem;

  public static boolean isDisabled = false;

  /** Creates a new Blinkin. */
  public Blinkin(StagingSubsystem stagingSubsystem) {
    blinkinController = new Spark(BlinkinConstants.kPwmPort);

    this.stagingSubsystem = stagingSubsystem;

    initialTime = 0.0;
    currentTime = 0.0;

    useReturnToRobotStateTimer = false;
    returnToRobotStateTimeLimit = 0.0;

    lastFlashTime = 0.0;
    useFlashColor = false;
    flashColor = 0.0;
    flashRate = 0.0;
    flashOn = false;

    currentColor = 0.0;

  }

  public void set(double value) {
    if (GlobalConstants.kUseLEDLights) {
      blinkinController.set(value);
      currentColor = value;
    }
    useReturnToRobotStateTimer = false;
  }

  public void neutral() {
    blinkinController.disable();
    // set(0.99);
  }

  // Solid purple color (not necessarily identical to color 2 preset)
  public void purple() {
    set(0.89);
  }

  // Solid gold color (not necessarily identical to color 1 preset)
  public void gold() {
    set(0.63);
  }

  // Solid pink color
  public void pink() {
    set(0.57);
  }

  // Solid green color
  public void green() {
    set(0.75);
  }

  // Solid red color
  public void red() {
    set(0.61);
  }

  // Solid aqua color
  public void aqua() {
    set(0.81);
  }

  // Solid orange color
  public void orange() {
    set(0.65);
  }

  public void rainbowTwinkle() {
    set(-0.55);
  }

  public void whiteOverride() {
    set(0.93);
  }

  public void goldHeartbeat() {
    set(0.15);
  }

  public void purpleHeartbeat() {
    set(0.35);
  }

  // Solid green (for various success modes)
  public void success() {
    green();
    returnToRobotState(1.5);
  }

  // Solid red (for various failure modes)
  public void failure() {
    red();
    returnToRobotState(1);
  }

  // Return to robot state after a certain number of seconds (color/pattern on
  // timer)
  public void returnToRobotState(double seconds) {
    initialTime = Timer.getFPGATimestamp();
    useReturnToRobotStateTimer = true;
    returnToRobotStateTimeLimit = seconds;
  }

  public void returnToRobotState() {
    useReturnToRobotStateTimer = false;
    useFlashColor = false;
    if (isDisabled) {
      aqua();
    } else if (stagingSubsystem.getState() == StagingState.DRIVING_INTAKE) {
      red();
    } else if (stagingSubsystem.getState() == StagingState.NOTE_INSIDE) {
      green();
      flashColorAtRate(5.0, 0.35);
    } else if (stagingSubsystem.getState() == StagingState.ASK_FOR_NOTE) {
      rainbowTwinkle();
    } else if (stagingSubsystem.getState() == StagingState.EMPTY) {
      orange();
    } else {
      System.out.println("Invalid logic - this line should not be hit");
    }
  }

  public static void setIsDisabled(boolean t) {
    isDisabled = t;
  }

  public void flashColorAtRate(double seconds, double colorCode) {
    lastFlashTime = Timer.getFPGATimestamp();
    useFlashColor = true;
    flashColor = colorCode;
    flashRate = seconds;
    flashOn = false;
  }

  @Override
  public void periodic() {
    if (GlobalConstants.kUseLEDLights) {
      SmartDashboard.putNumber("current LED color set", currentColor);
      SmartDashboard.putNumber("current LED color blinkin", blinkinController.get());
      SmartDashboard.putBoolean("isDisabled", isDisabled);
      // blinkin.set(.15);
      returnToRobotState();
      // if (useReturnToRobotStateTimer) {
      // currentTime = Timer.getFPGATimestamp();
      // if (currentTime - initialTime > returnToRobotStateTimeLimit) {
      // returnToRobotState();
      // }
      // }

      // if (useFlashColor) {
      // currentTime = Timer.getFPGATimestamp();
      // if (currentTime - lastFlashTime > flashRate) {
      // if (flashOn) {
      // blinkin.neutral();
      // lastFlashTime = Timer.getFPGATimestamp();
      // flashOn = false;
      // } else {
      // blinkin.set(flashColor);
      // lastFlashTime = Timer.getFPGATimestamp();
      // flashOn = true;
      // }
      // }
      // }
    }
  }
}
