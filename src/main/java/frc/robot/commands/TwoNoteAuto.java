package frc.robot.commands;

import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AutoLocationsConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.utils.PathFactory;

public class TwoNoteAuto extends SequentialCommandGroup {
    public TwoNoteAuto(DriveSubsystem m_robotDrive, PoseEstimatorSubsystem poseEstimatorSubsystem) {
        addCommands(Commands.runOnce(() -> {
            if (VisionConstants.kUsingVision) {
                poseEstimatorSubsystem.setToCurrentVisionPose();
            }
            Pose2d visionPose = poseEstimatorSubsystem.getCurrentPose();
        }),
                // go to shooting location
                PathFactory.followPath(poseEstimatorSubsystem.getCurrentPose(),
                        AutoLocationsConstants.kBlueShootLocation1, AutoLocationsConstants.kZeroDegreeRotation),
                // shoot
                new WaitCommand(2),
                // pick up another note
                PathFactory.followPath(poseEstimatorSubsystem.getCurrentPose(),
                        AutoLocationsConstants.kBlueStartingNote1, AutoLocationsConstants.kZeroDegreeRotation),
                // go to shooting location
                PathFactory.followPath(poseEstimatorSubsystem.getCurrentPose(),
                        AutoLocationsConstants.kBlueShootLocation1, AutoLocationsConstants.kZeroDegreeRotation),
                // shoot
                new WaitCommand(2),
                // pick up another note
                PathFactory.followPath(poseEstimatorSubsystem.getCurrentPose(),
                        AutoLocationsConstants.kBlueStartingNote1, AutoLocationsConstants.kZeroDegreeRotation),
                // go to shooting location
                PathFactory.followPath(poseEstimatorSubsystem.getCurrentPose(),
                        AutoLocationsConstants.kBlueShootLocation1, AutoLocationsConstants.kZeroDegreeRotation),
                // shoot
                new WaitCommand(2));
    }
}
