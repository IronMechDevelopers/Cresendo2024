package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.StagingSubsytem;

public class TwoNoteAuto extends SequentialCommandGroup {

        private DriveSubsystem driveSubsystem;

        public TwoNoteAuto(StagingSubsytem stagingSubsytem, ShooterSubsystem shooterSubsystem,
                        DriveSubsystem driveSubsystem) {
                this.driveSubsystem = driveSubsystem;
                addCommands(
                                new ParallelRaceGroup(
                                                new ShootWarmpUpCommand(shooterSubsystem), new WaitCommand(.25)
                                                                .andThen(new IntakeCommand(stagingSubsytem,
                                                                                Constants.SpeedConstants.InTakeSpeed)
                                                                                .withTimeout(.75))),
                                new ParallelRaceGroup(getDriveBackwardCommand().withTimeout(1.5),
                                                new DrivingIntake(stagingSubsytem)),
                                new ParallelRaceGroup(getDriveFowardCommand().withTimeout(1.75),
                                                new ShootWarmpUpCommand(shooterSubsystem)),
                                new ParallelRaceGroup(
                                                new ShootWarmpUpCommand(shooterSubsystem), new WaitCommand(.25)
                                                                .andThen(new IntakeCommand(stagingSubsytem,
                                                                                Constants.SpeedConstants.InTakeSpeed)
                                                                                .withTimeout(.75))));
        }

        public Command getDriveFowardCommand() {

                return Commands.startEnd(() -> driveSubsystem.drive(.25, 0, 0, false),
                                () -> driveSubsystem.drive(0, 0, 0, false), driveSubsystem);

        }

        public Command getDriveBackwardCommand() {
                return Commands.startEnd(() -> driveSubsystem.drive(-.25, 0, 0, false),
                                () -> driveSubsystem.drive(0, 0, 0, false), driveSubsystem);
        }

        // public Command getDriveFowardCommand() {
        // // Create config for trajectory
        // TrajectoryConfig config = new TrajectoryConfig(
        // AutoConstants.kMaxSpeedMetersPerSecond / 4.0,
        // AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // // Add kinematics to ensure max speed is actually obeyed
        // .setKinematics(DriveConstants.kDriveKinematics);

        // // An example trajectory to follow. All units in meters.
        // Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // // Start at the origin facing the +X direction
        // new Pose2d(0, 0, new Rotation2d(0)),
        // // Pass through these two interior waypoints, making an 's' curve path
        // List.of(new Translation2d(1, 0)),
        // // End 3 meters straight ahead of where we started, facing forward
        // new Pose2d(Units.feetToMeters(8), 0, new Rotation2d(0)),
        // config);

        // var thetaController = new ProfiledPIDController(
        // AutoConstants.kPThetaController, 0, 0,
        // AutoConstants.kThetaControllerConstraints);
        // thetaController.enableContinuousInput(-Math.PI, Math.PI);

        // SwerveControllerCommand swerveControllerCommand = new
        // SwerveControllerCommand(
        // exampleTrajectory,
        // driveSubsystem::getPose, // Functional interface to feed supplier
        // DriveConstants.kDriveKinematics,

        // // Position controllers
        // new PIDController(AutoConstants.kPXController, 0, 0),
        // new PIDController(AutoConstants.kPYController, 0, 0),
        // thetaController,
        // driveSubsystem::setModuleStates,
        // driveSubsystem);

        // // Reset odometry to the starting pose of the trajectory.
        // driveSubsystem.resetOdometry(exampleTrajectory.getInitialPose());

        // // Run path following command, then stop at the end.
        // return swerveControllerCommand.andThen(() -> driveSubsystem.drive(0, 0, 0,
        // false));
        // }

        // public Command getDriveBackwardCommand() {
        // // Create config for trajectory
        // TrajectoryConfig config = new TrajectoryConfig(
        // AutoConstants.kMaxSpeedMetersPerSecond / 4.0,
        // AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // // Add kinematics to ensure max speed is actually obeyed
        // .setKinematics(DriveConstants.kDriveKinematics);

        // // An example trajectory to follow. All units in meters.
        // Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // // Start at the origin facing the +X direction
        // new Pose2d(0, 0, new Rotation2d(0)),
        // // Pass through these two interior waypoints, making an 's' curve path
        // List.of(new Translation2d(-1, 0)),
        // // End 3 meters straight ahead of where we started, facing forward
        // new Pose2d(Units.feetToMeters(-8), 0, new Rotation2d(0)),
        // config);

        // var thetaController = new ProfiledPIDController(
        // AutoConstants.kPThetaController, 0, 0,
        // AutoConstants.kThetaControllerConstraints);
        // thetaController.enableContinuousInput(-Math.PI, Math.PI);

        // SwerveControllerCommand swerveControllerCommand = new
        // SwerveControllerCommand(
        // exampleTrajectory,
        // driveSubsystem::getPose, // Functional interface to feed supplier
        // DriveConstants.kDriveKinematics,

        // // Position controllers
        // new PIDController(AutoConstants.kPXController, 0, 0),
        // new PIDController(AutoConstants.kPYController, 0, 0),
        // thetaController,
        // driveSubsystem::setModuleStates,
        // driveSubsystem);

        // // Reset odometry to the starting pose of the trajectory.
        // driveSubsystem.resetOdometry(exampleTrajectory.getInitialPose());

        // // Run path following command, then stop at the end.
        // return swerveControllerCommand.andThen(() -> driveSubsystem.drive(0, 0, 0,
        // false));
        // }
}
