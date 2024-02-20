// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.TwoNoteAuto;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.StagingSubsytem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

        // The robot's subsystems
        private final DriveSubsystem m_robotDrive = new DriveSubsystem();
        private final StagingSubsytem m_StagingSubsytem = new StagingSubsytem();
        private final ShooterSubsystem m_ShooterSubsystem = new ShooterSubsystem();
        private static final Joystick driverLeftStick = new Joystick(0);
        private static final Joystick driverRightStick = new Joystick(1);
        private static final XboxController copilotXbox = new XboxController(2);

        private final JoystickButton right1Button = new JoystickButton(driverRightStick, 1);
        private final JoystickButton right3Button = new JoystickButton(driverRightStick, 3);
        private final JoystickButton right2Button = new JoystickButton(driverRightStick, 2);

        private final JoystickButton left1Button = new JoystickButton(driverLeftStick, 1);
        private final JoystickButton left4Button = new JoystickButton(driverLeftStick, 4);
        private final JoystickButton left2Button = new JoystickButton(driverLeftStick, 2);

        private final JoystickButton yButton = new JoystickButton(copilotXbox, Button.kY.value);
        private final JoystickButton xButton = new JoystickButton(copilotXbox, Button.kX.value);
        private final JoystickButton bButton = new JoystickButton(copilotXbox, Button.kB.value);
        private final JoystickButton aButton = new JoystickButton(copilotXbox, Button.kA.value);
        private final JoystickButton rightBumperButton = new JoystickButton(copilotXbox, Button.kRightBumper.value);
        private final JoystickButton leftBumperButton = new JoystickButton(copilotXbox, Button.kLeftBumper.value);

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {

                CameraServer.startAutomaticCapture();

                // Creates the CvSink and connects it to the UsbCamera
                CvSink cvSink = CameraServer.getVideo();

                // Creates the CvSource and MjpegServer [2] and connects them
                CvSource outputStream = CameraServer.putVideo("Shooting", 640, 480);

                DataLogManager.start();
                DriverStation.startDataLog(DataLogManager.getLog());

                // Configure the button bindings
                configureButtonBindings();
                SmartDashboard.putNumber("Fast Speed", .5);
                SmartDashboard.putNumber("Slow Speed", .18);

                // Configure default commands
                m_robotDrive.setDefaultCommand(
                                // The left stick controls translation of the robot.
                                // Turning is controlled by the X axis of the right stick.
                                new RunCommand(
                                                () -> m_robotDrive.drive(
                                                                -MathUtil.applyDeadband(driverLeftStick.getY(),
                                                                                OIConstants.kDriveDeadband),
                                                                -MathUtil.applyDeadband(driverLeftStick.getX(),
                                                                                OIConstants.kDriveDeadband),
                                                                -MathUtil.applyDeadband(driverRightStick.getX(),
                                                                                OIConstants.kDriveDeadband),
                                                                true),
                                                m_robotDrive));
        }

        /**
         * Use this method to define your button->command mappings. Buttons can be
         * created by
         * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
         * subclasses ({@link
         * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
         * passing it to a
         * {@link JoystickButton}.
         */
        private void configureButtonBindings() {
                right3Button.onTrue(m_robotDrive.zeroGyroCommand());
                left1Button.toggleOnTrue(m_StagingSubsytem.drivingIntakeCommand());

                left4Button.whileTrue(m_StagingSubsytem.runOuttakeCommand());
                right1Button.toggleOnTrue(m_StagingSubsytem.runIntakeCommand());

                rightBumperButton.toggleOnTrue(m_ShooterSubsystem.setMotorToPercentCommand("Fast Speed"));
                leftBumperButton.toggleOnTrue(m_ShooterSubsystem.setMotorToPercentCommand("Slow Speed"));
                xButton.toggleOnTrue(m_StagingSubsytem.drivingIntakeCommand());
                aButton.toggleOnTrue(m_StagingSubsytem.runIntakeCommand());
                bButton.toggleOnTrue(m_StagingSubsytem.runOuttakeCommand());
                right2Button.onTrue(m_robotDrive.switchMaxSpeedCommand());
                left2Button.whileTrue(m_robotDrive.setXCommand());

                SmartDashboard.putData("Invert Field Orientation", m_robotDrive.invertFieldRelativeComand());
                SmartDashboard.putBoolean("Field Orientation:", m_robotDrive.getFieldOrientation());
                SmartDashboard.putData("2 Note Auto", new TwoNoteAuto(m_StagingSubsytem, m_ShooterSubsystem,
                                m_robotDrive));
                SmartDashboard.putData("Follow Path_test", PathFollowing());

        }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {
                return new TwoNoteAuto(m_StagingSubsytem, m_ShooterSubsystem,
                                m_robotDrive);
                // // Create config for trajectory
                // TrajectoryConfig config = new TrajectoryConfig(
                // AutoConstants.kMaxSpeedMetersPerSecond,
                // AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                // // Add kinematics to ensure max speed is actually obeyed
                // .setKinematics(DriveConstants.kDriveKinematics);

                // // An example trajectory to follow. All units in meters.
                // Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
                // // Start at the origin facing the +X direction
                // new Pose2d(0, 0, new Rotation2d(0)),
                // // Pass through these two interior waypoints, making an 's' curve path
                // List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
                // // End 3 meters straight ahead of where we started, facing forward
                // new Pose2d(3, 0, new Rotation2d(0)),
                // config);

                // var thetaController = new ProfiledPIDController(
                // AutoConstants.kPThetaController, 0, 0,
                // AutoConstants.kThetaControllerConstraints);
                // thetaController.enableContinuousInput(-Math.PI, Math.PI);

                // SwerveControllerCommand swerveControllerCommand = new
                // SwerveControllerCommand(
                // exampleTrajectory,
                // m_robotDrive::getPose, // Functional interface to feed supplier
                // DriveConstants.kDriveKinematics,

                // // Position controllers
                // new PIDController(AutoConstants.kPXController, 0, 0),
                // new PIDController(AutoConstants.kPYController, 0, 0),
                // thetaController,
                // m_robotDrive::setModuleStates,
                // m_robotDrive);

                // // Reset odometry to the starting pose of the trajectory.
                // m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

                // // Run path following command, then stop at the end.
                // return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0,
                // false));
        }

        private PathPlannerPath getPath() {
                // Create a list of bezier points from poses. Each pose represents one waypoint.
                // The rotation component of the pose should be the direction of travel. Do not
                // use holonomic rotation.
                List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
                                new Pose2d(1.0, 1.0, Rotation2d.fromDegrees(0)),
                                new Pose2d(3.0, 1.0, Rotation2d.fromDegrees(0)),
                                new Pose2d(5.0, 3.0, Rotation2d.fromDegrees(90)));

                // Create the path using the bezier points created above
                PathPlannerPath path = new PathPlannerPath(
                                bezierPoints,
                                new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI), // The constraints for this
                                // path. If using a
                                // differential drivetrain, the
                                // angular constraints have no
                                // effect.
                                new GoalEndState(0.0, Rotation2d.fromDegrees(-90)) // Goal end state. You can set a
                                                                                   // holonomic rotation here. If using
                                                                                   // a differential drivetrain, the
                                                                                   // rotation will have no effect.
                );

                // Prevent the path from being flipped if the coordinates are already correct
                path.preventFlipping = true;
                return path;
        }

        private Command PathFollowing() {
                PathPlannerPath path = getPath();
                return AutoBuilder.followPath(path);
        }
}
