// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;
import java.util.Arrays;
import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.MotorIds;
import frc.utils.SwerveUtils;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.Vector;
import com.pathplanner.lib.auto.AutoBuilder;

public class DriveSubsystem extends SubsystemBase {
  // Create MAXSwerveModules
  private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
      MotorIds.kFrontLeftDrivingCanId,
      MotorIds.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset);

  private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
      MotorIds.kFrontRightDrivingCanId,
      MotorIds.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset);

  private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
      MotorIds.kRearLeftDrivingCanId,
      MotorIds.kRearLeftTurningCanId,
      DriveConstants.kBackLeftChassisAngularOffset);

  private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
      MotorIds.kRearRightDrivingCanId,
      MotorIds.kRearRightTurningCanId,
      DriveConstants.kBackRightChassisAngularOffset);

  // The gyro sensor
  private final ADIS16470_IMU m_gyro = new ADIS16470_IMU();

  private boolean fieldOrientation = true;

  // Slew rate filter variables for controlling lateral acceleration
  private double m_currentRotation = 0.0;
  private double m_currentTranslationDir = 0.0;
  private double m_currentTranslationMag = 0.0;

  private SlewRateLimiter m_magLimiter = new SlewRateLimiter(DriveConstants.kMagnitudeSlewRate);
  private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(DriveConstants.kRotationalSlewRate);

  private double m_prevTime = WPIUtilJNI.now() * 1e-6;
  private final Field2d field2d = new Field2d();
  private final AprilTagFieldLayout aprilTagFieldLayout;
  private volatile Pose2d visionPose2d;
  private final SwerveDrivePoseEstimator poseEstimator;
  private final MAXSwerveModule[] swerveModules;

  private boolean isFullSpeed = true;

  /**
   * Standard deviations of model states. Increase these numbers to trust your
   * model's state estimates less. This
   * matrix is in the form [x, y, theta]ᵀ, with units in meters and radians, then
   * meters.
   */
  private static final Vector<N3> stateStdDevs = VecBuilder.fill(0.1, 0.1, 0.1);

  /**
   * Standard deviations of the vision measurements. Increase these numbers to
   * trust global measurements from vision
   * less. This matrix is in the form [x, y, theta]ᵀ, with units in meters and
   * radians.
   */
  private static final Vector<N3> visionMeasurementStdDevs = VecBuilder.fill(1.5, 1.5, 1.5);

  private SwerveDriveKinematics kinematics = DriveConstants.kDriveKinematics;

  StructPublisher<Pose2d> myPosePublisher = NetworkTableInstance.getDefault()
      .getStructTopic("MyPose", Pose2d.struct).publish();

  StructArrayPublisher<SwerveModuleState> mySwerveStatesPublisher = NetworkTableInstance.getDefault()
      .getStructArrayTopic("MyStates", SwerveModuleState.struct).publish();

  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
      DriveConstants.kDriveKinematics,
      Rotation2d.fromDegrees(m_gyro.getAngle(IMUAxis.kZ)),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      });

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    AprilTagFieldLayout layout;

    try {
      layout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
      Optional<Alliance> alliance = DriverStation.getAlliance();
      layout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
    } catch (IOException e) {
      DriverStation.reportError("Failed to load AprilTagFieldLayout", e.getStackTrace());
      layout = null;
    }

    this.aprilTagFieldLayout = layout;

    swerveModules = new MAXSwerveModule[] { m_frontLeft, m_frontRight, m_rearLeft, m_rearRight };

    poseEstimator = new SwerveDrivePoseEstimator(
        DriveConstants.kDriveKinematics,
        getGyroscopeRotation(),
        getModulePositions(),
        new Pose2d(),
        stateStdDevs,
        visionMeasurementStdDevs);

    AutoBuilder.configureHolonomic(
        this::getCurrentPose,
        this::setCurrentPose,
        this::getSpeeds,
        this::driveRobotRelative,
        Constants.DriveConstants.pathFollowerConfig,
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red
          // alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          Optional<Alliance> alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this);

  }

  public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
    ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);

    SwerveModuleState[] targetStates = kinematics.toSwerveModuleStates(targetSpeeds);
    setStates(targetStates);
  }

  public void setStates(SwerveModuleState[] targetStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(targetStates, Constants.AutoConstants.kMaxSpeedMetersPerSecond);

    for (int i = 0; i < swerveModules.length; i++) {
      swerveModules[i].setDesiredState(targetStates[i]);
    }
  }

  public Pose2d getCurrentPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /**
   * Resets the current pose to the specified pose. This should ONLY be called
   * when the robot's position on the field is known, like at the beginning of
   * a match.
   * 
   * @param newPose new pose
   */
  public void setCurrentPose(Pose2d newPose) {
    poseEstimator.resetPosition(
        getGyroscopeRotation(),
        getModulePositions(),
        newPose);
  }

  /**
   * Gets the current drivetrain position, as reported by the modules themselves.
   * 
   * @return current drivetrain state. Array orders are frontLeft, frontRight,
   *         backLeft, backRight
   */
  public SwerveModulePosition[] getModulePositions() {
    return Arrays.stream(swerveModules).map(module -> module.getPosition()).toArray(SwerveModulePosition[]::new);
  }

  /**
   * Gets the current drivetrain state (velocity, and angle), as reported by the
   * modules themselves.
   * 
   * @return current drivetrain state. Array orders are frontLeft, frontRight,
   *         backLeft, backRight
   */
  private SwerveModuleState[] getModuleStates() {
    return Arrays.stream(swerveModules).map(module -> module.getState()).toArray(SwerveModuleState[]::new);
  }

  @Override
  public void periodic() {
    poseEstimator.update(getGyroscopeRotation(), getModulePositions());

    Pose2d pose = poseEstimator.getEstimatedPosition();
    SmartDashboard.putBoolean("is Inverted", fieldOrientation);
    SmartDashboard.putString("Pose", getFormattedPose());
    field2d.setRobotPose(getCurrentPose());

    myPosePublisher.set(pose);
    mySwerveStatesPublisher.set(getModuleStates());

  }

  private String getFormattedPose() {
    Pose2d pose = getCurrentPose();
    return String.format("(%.2f, %.2f) %.2f degrees",
        pose.getX(),
        pose.getY(),
        pose.getRotation().getDegrees());
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed           Speed of the robot in the x direction (forward).
   * @param ySpeed           Speed of the robot in the y direction (sideways).
   * @param rot              Angular rate of the robot.
   * @param fieldOrientation Whether the provided x and y speeds are relative to
   *                         the
   *                         field.
   * @param rateLimit        Whether to enable rate limiting for smoother control.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean rateLimit) {

    double xSpeedCommanded;
    double ySpeedCommanded;

    if (rateLimit) {
      // Convert XY to polar for rate limiting
      double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
      double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

      // Calculate the direction slew rate based on an estimate of the lateral
      // acceleration
      double directionSlewRate;
      if (m_currentTranslationMag != 0.0) {
        directionSlewRate = Math.abs(DriveConstants.kDirectionSlewRate / m_currentTranslationMag);
      } else {
        directionSlewRate = 500.0; // some high number that means the slew rate is effectively instantaneous
      }

      double currentTime = WPIUtilJNI.now() * 1e-6;
      double elapsedTime = currentTime - m_prevTime;
      double angleDif = SwerveUtils.AngleDifference(inputTranslationDir, m_currentTranslationDir);
      if (angleDif < 0.45 * Math.PI) {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir,
            directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
      } else if (angleDif > 0.85 * Math.PI) {
        if (m_currentTranslationMag > 1e-4) { // some small number to avoid floating-point errors with equality checking
          // keep currentTranslationDir unchanged
          m_currentTranslationMag = m_magLimiter.calculate(0.0);
        } else {
          m_currentTranslationDir = SwerveUtils.WrapAngle(m_currentTranslationDir + Math.PI);
          m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
        }
      } else {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir,
            directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(0.0);
      }
      m_prevTime = currentTime;

      xSpeedCommanded = m_currentTranslationMag * Math.cos(m_currentTranslationDir);
      ySpeedCommanded = m_currentTranslationMag * Math.sin(m_currentTranslationDir);
      m_currentRotation = m_rotLimiter.calculate(rot);

    } else {
      xSpeedCommanded = xSpeed;
      ySpeedCommanded = ySpeed;
      m_currentRotation = rot;
    }

    double maxSpeed = 0;
    if (isFullSpeed) {
      maxSpeed = DriveConstants.kMaxSpeedMetersPerSecond;
    } else {
      maxSpeed = DriveConstants.kHalfSpeedMetersPerSecond;
    }

    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeedCommanded * maxSpeed;
    double ySpeedDelivered = ySpeedCommanded * maxSpeed;
    double rotDelivered = m_currentRotation * DriveConstants.kMaxAngularSpeed;

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldOrientation
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
                Rotation2d.fromDegrees(m_gyro.getAngle(IMUAxis.kZ)))
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, maxSpeed);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return Rotation2d.fromDegrees(m_gyro.getAngle(IMUAxis.kZ)).getDegrees();
  }

  public Rotation2d getGyroscopeRotation() {
    return Rotation2d.fromDegrees(m_gyro.getAngle(IMUAxis.kZ));

    // We have to invert the angle of the NavX so that rotating the robot
    // counter-clockwise makes the angle increase.
    // return Rotation2d.fromDegrees(360.0 - navx.getYaw());
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate(IMUAxis.kZ) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  public ChassisSpeeds getSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  /**
   * Inverts field relatives value
   *
   */
  public void invertFieldOrientation() {
    fieldOrientation = !fieldOrientation;
  }

  /**
   * Returns the field orientation.
   *
   * @return The field orientation as a boolean.
   */
  public boolean getFieldOrientation() {
    return fieldOrientation;
  }

  public void switchMaxSpeed() {
    isFullSpeed = !isFullSpeed;
  }

  /**
   * @return
   */
  public Command zeroGyroCommand() {

    return Commands.runOnce(() -> zeroHeading(), this);
  }

  public Command setXCommand() {
    return Commands.run(() -> setX(), this);
  }

  public Command switchMaxSpeedCommand() {
    return Commands.runOnce(() -> switchMaxSpeed());
  }

  public Command invertFieldRelativeComand() {
    return Commands.runOnce(() -> invertFieldOrientation(), this);
  }

  public Command driveCommand(double x, double y, double rot) {
    return Commands.startEnd(() -> drive(x, y, rot, true),
        () -> drive(0, 0, 0, true), this);
  }
}