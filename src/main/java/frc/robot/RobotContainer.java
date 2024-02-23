// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.TwoNoteAuto;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.StagingSubsytem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
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
        private final JoystickButton right2Button = new JoystickButton(driverRightStick, 2);
        private final JoystickButton right3Button = new JoystickButton(driverRightStick, 3);

        private final JoystickButton left1Button = new JoystickButton(driverLeftStick, 1);
        private final JoystickButton left4Button = new JoystickButton(driverLeftStick, 4);
        private final JoystickButton left2Button = new JoystickButton(driverLeftStick, 2);

        private final JoystickButton xButton = new JoystickButton(copilotXbox, Button.kX.value);
        private final JoystickButton bButton = new JoystickButton(copilotXbox, Button.kB.value);
        private final JoystickButton aButton = new JoystickButton(copilotXbox, Button.kA.value);
        private final JoystickButton rightBumperButton = new JoystickButton(copilotXbox, Button.kRightBumper.value);
        private final JoystickButton leftBumperButton = new JoystickButton(copilotXbox, Button.kLeftBumper.value);

        private final SendableChooser<Command> autoChooser;

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {

                autoChooser = AutoBuilder.buildAutoChooser();

                CameraServer.startAutomaticCapture();

                // Creates the CvSink and connects it to the UsbCamera
                CvSink cvSink = CameraServer.getVideo();

                // Creates the CvSource and MjpegServer [2] and connects them
                CvSource outputStream = CameraServer.putVideo("Shooting", 640, 480);

                DataLogManager.start();
                DriverStation.startDataLog(DataLogManager.getLog());

                NamedCommands.registerCommand("intake", m_StagingSubsytem.drivingIntakeCommand().withTimeout(5));
                NamedCommands.registerCommand("shootHigh", Commands.race(
                                m_ShooterSubsystem.setMotorToPercentCommand("Fast Speed"),
                                new WaitCommand(.25).andThen(m_StagingSubsytem.runIntakeCommand().withTimeout(1))));
                NamedCommands.registerCommand("WarmUpShooter",
                                m_ShooterSubsystem.setMotorToPercentCommand("Fast Speed"));

                autoChooser.setDefaultOption("Do Nothing", new WaitCommand(15));
                autoChooser.addOption("Taxi", m_robotDrive.driveCommand(-.25, 0, 0).withTimeout(2));
                autoChooser.addOption("Center-No Move", Commands.race(
                                m_ShooterSubsystem.setMotorToPercentCommand("Fast Speed"),
                                new WaitCommand(.25).andThen(m_StagingSubsytem.runIntakeCommand().withTimeout(1))));
                autoChooser.addOption("Center-Center", new PathPlannerAuto("Center-Center"));
                autoChooser.addOption("Center-Center-Amp", new PathPlannerAuto("Center-Center-Amp"));
                autoChooser.addOption("Amp-No Move", new WaitCommand(15));
                autoChooser.addOption("Amp-Amp", new WaitCommand(15));
                autoChooser.addOption("Amp-Amp-Cross field", new WaitCommand(15));
                autoChooser.addOption("Troll", new WaitCommand(15));

                autoChooser.addOption("Drive Forward", m_robotDrive.driveCommand(.25, 0, 0).withTimeout(2));
                autoChooser.addOption("2 Note Center", new PathPlannerAuto("2NoteCenter"));
                autoChooser.addOption("2 Note Right Far Right", new PathPlannerAuto("2NoteRightFarRight"));
                autoChooser.addOption("3 Note Center Left", new PathPlannerAuto("3NoteCenterLeft"));

                SmartDashboard.putData("Auto", autoChooser);

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
                                                                false),
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
                ;
                left1Button.toggleOnTrue(m_StagingSubsytem.drivingIntakeCommand());
                left2Button.whileTrue(m_robotDrive.setXCommand());
                left4Button.whileTrue(m_StagingSubsytem.runOuttakeCommand());

                right1Button.toggleOnTrue(m_StagingSubsytem.runIntakeCommand());
                right2Button.onTrue(m_robotDrive.switchMaxSpeedCommand());
                right3Button.onTrue(m_robotDrive.zeroGyroCommand());

                rightBumperButton.toggleOnTrue(m_ShooterSubsystem.setMotorToPercentCommand("Fast Speed"));
                leftBumperButton.toggleOnTrue(m_ShooterSubsystem.setMotorToPercentCommand("Slow Speed"));
                xButton.toggleOnTrue(m_StagingSubsytem.drivingIntakeCommand());
                aButton.toggleOnTrue(m_StagingSubsytem.runIntakeCommand());
                bButton.toggleOnTrue(m_StagingSubsytem.runOuttakeCommand());

                SmartDashboard.putData("Invert Field Orientation", m_robotDrive.invertFieldRelativeComand());
                SmartDashboard.putBoolean("Field Orientation:", m_robotDrive.getFieldOrientation());
                SmartDashboard.putData("2 Note Auto", new TwoNoteAuto(m_StagingSubsytem, m_ShooterSubsystem,
                                m_robotDrive));

                SmartDashboard.putData("3 Note left Auto", new PathPlannerAuto("3NoteLeftAuto"));
                SmartDashboard.putData("Simple Test", new PathPlannerAuto("Simple"));
                SmartDashboard.putData("3NoteCenterLeft", new PathPlannerAuto("3NoteCenterLeft"));

        }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {
                return autoChooser.getSelected();
        }
}
