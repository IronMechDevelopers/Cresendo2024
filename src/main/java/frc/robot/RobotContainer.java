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
import frc.robot.commands.TestingSubsystemsCommand;
import frc.robot.subsystems.AmpFlopper;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PowerDistributionModule;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.StagingSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

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
        private final StagingSubsystem m_StagingSubsystem = new StagingSubsystem();
        private final ShooterSubsystem m_ShooterSubsystem = new ShooterSubsystem();
        private final ClimberSubsystem m_ClimberSubsystem = new ClimberSubsystem();
        private final AmpFlopper m_AmpFlopper = new AmpFlopper();
        private final PowerDistributionModule powerDistributionModule = new PowerDistributionModule();

        private static final Joystick driverLeftStick = new Joystick(0);
        private static final Joystick driverRightStick = new Joystick(1);
        private static final XboxController copilotXbox = new XboxController(2);

        private final JoystickButton left1Button = new JoystickButton(driverLeftStick, 1);
        private final JoystickButton left2Button = new JoystickButton(driverLeftStick, 2);

        private final JoystickButton right1Button = new JoystickButton(driverRightStick, 1);
        private final JoystickButton right2Button = new JoystickButton(driverRightStick, 2);
        private final JoystickButton right3Button = new JoystickButton(driverRightStick, 3);
        private final JoystickButton right4Button = new JoystickButton(driverRightStick, 4);
        private final JoystickButton right6Button = new JoystickButton(driverRightStick, 6);

        private final JoystickButton xButton = new JoystickButton(copilotXbox, Button.kX.value);
        private final JoystickButton bButton = new JoystickButton(copilotXbox, Button.kB.value);
        private final JoystickButton aButton = new JoystickButton(copilotXbox, Button.kA.value);
        private final JoystickButton yButton = new JoystickButton(copilotXbox, Button.kY.value);
        private final JoystickButton rightBumperButton = new JoystickButton(copilotXbox, Button.kRightBumper.value);
        private final JoystickButton leftBumperButton = new JoystickButton(copilotXbox, Button.kLeftBumper.value);
        private final Trigger leftTigger = new Trigger(() -> copilotXbox.getLeftTriggerAxis() > .5);
        private final Trigger rightTigger = new Trigger(() -> copilotXbox.getRightTriggerAxis() > .5);

        private SendableChooser<Command> auto = new SendableChooser<>();

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

                NamedCommands.registerCommand("intake", m_StagingSubsystem.drivingIntakeCommand().withTimeout(5));
                NamedCommands.registerCommand("shootHigh", Commands.race(
                                m_ShooterSubsystem.setMotorToPercentCommand("Fast Speed"),
                                m_StagingSubsystem.runIntakeCommand().withTimeout(1)));
                NamedCommands.registerCommand("WarmUpShooter",
                                m_ShooterSubsystem.warmUpMotorToPercentCommand("Fast Speed"));
                NamedCommands.registerCommand("ArmDown", m_AmpFlopper.ampFlopperDownCommand().withTimeout(1));

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

                left1Button.whileTrue(m_ShooterSubsystem.setMotorToInvesePercentCommand());
                left2Button.whileTrue(m_robotDrive.setXCommand());

                right1Button.whileTrue(m_ClimberSubsystem.climberUpCommand());
                right2Button.onTrue(m_robotDrive.switchMaxSpeedCommand());
                right3Button.onTrue(m_robotDrive.zeroGyroCommand());
                right4Button.whileTrue(m_ClimberSubsystem.climberDownCommand());
                right6Button.whileTrue(m_ClimberSubsystem.climberUpCommand());

                rightBumperButton.toggleOnTrue(m_ShooterSubsystem.setMotorToPercentCommand("Fast Speed")
                                .finallyDo(() -> m_StagingSubsystem.changeColorCommmand(255, 0, 0)));
                leftBumperButton.toggleOnTrue(m_ShooterSubsystem.setMotorToPercentCommand("Slow Speed")
                                .finallyDo(() -> m_StagingSubsystem.changeColorCommmand(255, 0, 0)));
                xButton.toggleOnTrue(m_StagingSubsystem.drivingIntakeCommand());
                aButton.toggleOnTrue(m_StagingSubsystem.runIntakeCommand());
                bButton.toggleOnTrue(m_StagingSubsystem.runOuttakeCommand());
                yButton.whileTrue(m_ClimberSubsystem.climberUpCommand());
                leftTigger.whileTrue(m_AmpFlopper.ampFlopperUpCommand());
                rightTigger.whileTrue(m_AmpFlopper.ampFlopperDownCommand());

                SmartDashboard.putData("Invert Field Orientation", m_robotDrive.invertFieldRelativeComand());
                SmartDashboard.putData("DANIEL USE ONLY",
                                new TestingSubsystemsCommand(m_robotDrive, m_StagingSubsystem, m_ShooterSubsystem));
                SmartDashboard.putData("Start Match", getStartCommand());

                // SmartDashboard.putData("Reset Autos:", Commands.runOnce(() -> createAuto()));

                createAuto();

        }

        public void createAuto() {
                auto = new SendableChooser<>();
                auto.setDefaultOption("Do Nothing", new WaitCommand(15));
                auto.addOption("Taxi Forward", new PathPlannerAuto("Taxi Forward"));
                auto.addOption("Taxi Backward", new PathPlannerAuto("Taxi Backward"));
                auto.addOption("Center-No Move", new PathPlannerAuto("Center-No Move"));
                auto.addOption("Center-Center", new PathPlannerAuto("Center-Center"));
                auto.addOption("Center-Center-Amp", new PathPlannerAuto("Center-Center-Amp"));
                auto.addOption("Amp-No Move", new PathPlannerAuto("Amp-No Move"));
                auto.addOption("Amp-Wait-Taxi", new PathPlannerAuto("Amp-Wait-Taxi"));
                auto.addOption("Amp-Amp", new PathPlannerAuto("Amp-Amp"));
                auto.addOption("Amp-Amp-Cross field", new PathPlannerAuto("Amp-Amp-Cross field"));
                auto.addOption("Source-No Move", new PathPlannerAuto("Source-No Move"));
                auto.addOption("Source-Cross field", new PathPlannerAuto("Source-Cross field"));
                auto.addOption("Troll", new PathPlannerAuto("Troll"));
                

                SmartDashboard.putData("Autonomous Command", auto);
        }

        private Command getStartCommand() {
                return Commands.parallel(m_ShooterSubsystem.warmUpMotorToPercentCommand("Fast Speed"),
                                m_AmpFlopper.ampFlopperDownCommand()).withTimeout(.25);
        }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {
                Command command = Commands.sequence(getStartCommand(), new WaitCommand(.5), auto.getSelected());
                createAuto();
                return command;
        }
}
