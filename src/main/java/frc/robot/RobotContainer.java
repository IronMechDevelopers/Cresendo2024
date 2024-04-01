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
import frc.robot.Constants.VolumeConstants;
import frc.robot.commands.TestingSubsystemsCommand;
import frc.robot.commands.TurnOffFieldOrientCommand;
import frc.robot.commands.VibrateController;
import frc.robot.commands.VolumeAngleCommand;
import frc.robot.subsystems.AmpFlopper;
import frc.robot.subsystems.Blinkin;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PowerDistributionModule;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.StagingSubsystem;
import frc.robot.subsystems.VolumeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.Optional;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

        // The robot's subsystems
        private final DriveSubsystem m_robotDrive = new DriveSubsystem();
        private final StagingSubsystem m_StagingSubsystem = new StagingSubsystem();
        private final ShooterSubsystem m_ShooterSubsystem = new ShooterSubsystem();
        private final ClimberSubsystem m_ClimberSubsystem = new ClimberSubsystem();
        private final AmpFlopper m_AmpFlopper = new AmpFlopper();
        private final VolumeSubsystem m_volume = new VolumeSubsystem();
        private final PowerDistributionModule powerDistributionModule = new PowerDistributionModule();

        private final Blinkin blinkn = new Blinkin(m_StagingSubsystem);

        private static final Joystick driverLeftStick = new Joystick(0);
        private static final Joystick driverRightStick = new Joystick(1);
        private static final XboxController copilotXbox = new XboxController(2);

        private final JoystickButton left1Button = new JoystickButton(driverLeftStick, 1);
        private final JoystickButton left2Button = new JoystickButton(driverLeftStick, 2);
        private final JoystickButton left3Button = new JoystickButton(driverLeftStick, 3);
        private final JoystickButton left4Button = new JoystickButton(driverLeftStick, 4);

        private final JoystickButton left7Button = new JoystickButton(driverLeftStick, 7);
        private final JoystickButton left8Button = new JoystickButton(driverLeftStick, 8);
        private final JoystickButton left9Button = new JoystickButton(driverLeftStick, 9);
        private final JoystickButton left10Button = new JoystickButton(driverLeftStick, 10);

        private final JoystickButton right1Button = new JoystickButton(driverRightStick, 1);
        private final JoystickButton right2Button = new JoystickButton(driverRightStick, 2);
        private final JoystickButton right3Button = new JoystickButton(driverRightStick, 3);
        private final JoystickButton right4Button = new JoystickButton(driverRightStick, 4);
        private final JoystickButton right6Button = new JoystickButton(driverRightStick, 6);

        private final JoystickButton right9Button = new JoystickButton(driverRightStick, 9);
        private final JoystickButton right10Button = new JoystickButton(driverRightStick, 10);

        private final JoystickButton xButton = new JoystickButton(copilotXbox, Button.kX.value);
        private final JoystickButton bButton = new JoystickButton(copilotXbox, Button.kB.value);
        private final JoystickButton aButton = new JoystickButton(copilotXbox, Button.kA.value);
        private final JoystickButton yButton = new JoystickButton(copilotXbox, Button.kY.value);

        private final JoystickButton backButton = new JoystickButton(copilotXbox, Button.kBack.value);
        private final JoystickButton startButton = new JoystickButton(copilotXbox, Button.kStart.value);
        private final JoystickButton rightBumperButton = new JoystickButton(copilotXbox, Button.kRightBumper.value);
        private final JoystickButton leftBumperButton = new JoystickButton(copilotXbox, Button.kLeftBumper.value);
        private final Trigger leftTigger = new Trigger(
                        () -> copilotXbox.getLeftTriggerAxis() > .5 && copilotXbox.getRightTriggerAxis() < .15);
        private final Trigger rightTigger = new Trigger(
                        () -> copilotXbox.getRightTriggerAxis() > .5 && copilotXbox.getLeftTriggerAxis() < .15);
        private final Trigger bothTigger = new Trigger(
                        () -> copilotXbox.getRightTriggerAxis() > .5 && copilotXbox.getLeftTriggerAxis() > .5);

        private final Trigger noteInsideTrigger = new Trigger(() -> m_StagingSubsystem.isNoteAtUpperSensor());

        private SendableChooser<Command> auto = new SendableChooser<>();

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {

                blinkn.aqua();

                // CameraServer.startAutomaticCapture();

                // Creates the CvSink and connects it to the UsbCamera
                // CvSink cvSink = CameraServer.getVideo();

                // Creates the CvSource and MjpegServer [2] and connects them
                // CvSource outputStream = CameraServer.putVideo("Shooting", 640, 480);

                DataLogManager.start();
                DriverStation.startDataLog(DataLogManager.getLog());

                NamedCommands.registerCommand("intake", m_StagingSubsystem.drivingIntakeCommand().withTimeout(5));
                NamedCommands.registerCommand("shootHigh", Commands.race(
                                m_ShooterSubsystem.setMotorToPercentCommand("Fast Speed"),
                                m_StagingSubsystem.runIntakeCommand().withTimeout(.5)));
                NamedCommands.registerCommand("shootLow", Commands.race(
                                m_ShooterSubsystem.setMotorToPercentCommand("Slow Speed"),
                                m_StagingSubsystem.runIntakeCommand().withTimeout(1)));
                NamedCommands.registerCommand("WarmUpShooter",
                                m_ShooterSubsystem.warmUpMotorToPercentCommand("Fast Speed"));
                NamedCommands.registerCommand("ArmDown", m_AmpFlopper.ampFlopperDownCommand().withTimeout(.15));

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
                // m_volume.setDefaultCommand(new VolumeAngleCommand(m_volume, VolumeConstants.kVolumeDownAngle));
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

                left1Button.whileTrue(m_ShooterSubsystem.setMotorToInvesePercentCommand());
                left2Button.whileTrue(m_robotDrive.setXCommand());
                left3Button.whileTrue(m_StagingSubsystem.askForNote());
                left7Button.onTrue(new VolumeAngleCommand(m_volume, VolumeConstants.kVolumeAmpAngle).withTimeout(.5));
                left9Button.onTrue(new VolumeAngleCommand(m_volume, VolumeConstants.kVolumeDownAngle).withTimeout(.5));

                

                right1Button.whileTrue(new TurnOffFieldOrientCommand(m_robotDrive));
                right2Button.onTrue(m_robotDrive.switchMaxSpeedCommand());
                right3Button.onTrue(m_robotDrive.zeroGyroCommand());
                right4Button.whileTrue(m_ClimberSubsystem.climberDownCommand());
                right6Button.whileTrue(m_ClimberSubsystem.climberUpCommand());
                right9Button.whileTrue(m_volume.putVolumeDown());
                right10Button.whileTrue(m_volume.putVolumeUp());

                rightBumperButton.whileTrue(m_ShooterSubsystem.setMotorToPercentCommand("Fast Speed"));
                leftBumperButton.whileTrue(Commands.parallel(m_ShooterSubsystem.setMotorToPercentCommand("Slow Speed"),
                               new VolumeAngleCommand(m_volume, VolumeConstants.kVolumeAmpAngle).withTimeout(1)));
                leftBumperButton.toggleOnFalse(new VolumeAngleCommand(m_volume, VolumeConstants.kVolumeDownAngle).withTimeout(.5));
                // leftBumperButton.whileTrue(m_ShooterSubsystem.setMotorToPercentCommand("Slow Speed"));
                xButton.whileTrue(m_StagingSubsystem.drivingIntakeCommand());
                aButton.toggleOnTrue(m_StagingSubsystem.runIntakeCommand());
                bButton.whileTrue(m_StagingSubsystem.runOuttakeCommand());
                yButton.whileTrue(m_ClimberSubsystem.climberUpCommand());
                leftTigger.whileTrue(m_AmpFlopper.ampFlopperUpCommand());
                rightTigger.whileTrue(m_AmpFlopper.ampFlopperDownCommand());
                startButton.whileTrue(m_ShooterSubsystem.setMotorToPercentCommand(-.5));
                backButton.whileTrue(m_ShooterSubsystem.setMotorToOneHundredPercentCommand());

                noteInsideTrigger.onTrue(new VibrateController(copilotXbox).withTimeout(2));

                SmartDashboard.putData("DANIEL USE ONLY",
                                new TestingSubsystemsCommand(m_robotDrive, m_StagingSubsystem, m_ShooterSubsystem));
                createAuto();

        }

        public void createAuto() {
                auto = new SendableChooser<>();

                auto.setDefaultOption("Center-No Move - (1)", new PathPlannerAuto("Center-No Move"));
                auto.addOption("Do Nothing - (0)", new WaitCommand(15));
                auto.addOption("Taxi Forward - (0)", new PathPlannerAuto("Taxi Forward"));
                auto.addOption("Taxi Backward - (0)", new PathPlannerAuto("Taxi Backward"));
                auto.addOption("Center-Center - (2)", new PathPlannerAuto("Center-Center"));
                auto.addOption("Center-Center-Amp - (3)", new PathPlannerAuto("Center-Center-Amp"));
                auto.addOption("Center-Center-Source - (3)", new PathPlannerAuto("Center-Center-Source"));
                auto.addOption("Four-Note - (4)", new PathPlannerAuto("Four-Note"));
                auto.addOption("Amp-No Move - (1)", new PathPlannerAuto("Amp-No Move"));
                auto.addOption("Amp-Wait-Taxi - (1)", new PathPlannerAuto("Amp-Wait-Taxi"));
                auto.addOption("Amp-Amp - (2)", new PathPlannerAuto("Amp-Amp"));
                auto.addOption("Source-No Move - (1)", new PathPlannerAuto("Source-No Move"));
                auto.addOption("Source-Wait-Taxi - (1)", new PathPlannerAuto("Source-Cross field"));
                auto.addOption("Source-Source - (2)", new PathPlannerAuto("Source-Source"));
                auto.addOption("Source-Center1 - (2)", new PathPlannerAuto("Source-Center1"));
                auto.addOption("Source-Center2 - (2)", new PathPlannerAuto("Source-Center2"));
                auto.addOption("Troll - (0)", new PathPlannerAuto("Troll"));

                SmartDashboard.putData("Autonomous Command", auto);
        }

        private Command getStartCommand() {
                return Commands.parallel(m_ShooterSubsystem.warmUpMotorToPercentCommand("Fast Speed"),
                                m_AmpFlopper.ampFlopperDownCommand(), m_volume.putVolumeDown()).withTimeout(.25);
        }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {
                // Optional<Alliance> alliance = DriverStation.getAlliance();
                // if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
                // m_robotDrive.setFlipped(true);
                // } else {
                // m_robotDrive.setFlipped(false);
                // }
                Command command = Commands.sequence(getStartCommand(), auto.getSelected());
                createAuto();
                return command;
        }
}
