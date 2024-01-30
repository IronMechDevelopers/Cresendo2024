package frc.robot.subsystems;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Optional;

import org.opencv.calib3d.Calib3d;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point;
import org.opencv.core.Point3;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.apriltag.AprilTagDetection;
import edu.wpi.first.apriltag.AprilTagDetector;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.apriltag.AprilTagPoseEstimator;
import edu.wpi.first.apriltag.AprilTagPoseEstimator.Config;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.ComputerVisionUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.VisionConstants;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.CoordinateSystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public class PoseEstimatorSubsystem extends SubsystemBase {
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

    private final SwerveDrivePoseEstimator poseEstimator;
    private DriveSubsystem driveSubsystem;
    private final Field2d field2d = new Field2d();
    private final AprilTagFieldLayout aprilTagFieldLayout;
    private volatile Pose2d visionPose2d;

    private static Thread visionThread;
    private ShuffleboardTab softwareTab = Shuffleboard.getTab("Software");

    public PoseEstimatorSubsystem(DriveSubsystem driveSubsystem) {
        this.driveSubsystem = driveSubsystem;
        AprilTagFieldLayout layout;

        try {
            layout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
            Optional<Alliance> alliance = DriverStation.getAlliance();
            layout.setOrigin(
                    alliance.isPresent() && alliance.get() == Alliance.Blue ? OriginPosition.kBlueAllianceWallRightSide
                            : OriginPosition.kRedAllianceWallRightSide);
        } catch (IOException e) {
            DriverStation.reportError("Failed to load AprilTagFieldLayout", e.getStackTrace());
            layout = null;
        }

        this.aprilTagFieldLayout = layout;

        poseEstimator = new SwerveDrivePoseEstimator(
                DriveConstants.kDriveKinematics,
                driveSubsystem.getGyroscopeRotation(),
                driveSubsystem.getModulePositions(),
                new Pose2d(),
                stateStdDevs,
                visionMeasurementStdDevs);

        ShuffleboardTab tab = Shuffleboard.getTab("Vision");
        tab.addString("Pose", this::getFomattedPose).withPosition(0, 0).withSize(2, 0);
        tab.add("Field", field2d).withPosition(2, 0).withSize(6, 4);

        if (VisionConstants.kUsingVision) {
            visionThread = new Thread(this::apriltagVisionThreadProc);
            visionThread.setDaemon(true);
            visionThread.start();
        }

    }

    @Override
    public void periodic() {
        // Update pose estimator with drivetrain sensors
        poseEstimator.update(
                driveSubsystem.getGyroscopeRotation(),
                driveSubsystem.getModulePositions());

        // poseEstimator.updateWithTime(Timer.getFPGATimestamp(),
        // driveSubsystem.getGyroscopeRotation(),
        // driveSubsystem.getModulePositions());
        ShuffleboardTab tab = Shuffleboard.getTab("Vision");
        tab.addString("Pose", this::getFomattedPose).withPosition(0, 0).withSize(2, 0);
        field2d.setRobotPose(getCurrentPose());

    }

    private String getFomattedPose() {
        Pose2d pose = getCurrentPose();
        return String.format("(%.2f, %.2f) %.2f degrees",
                pose.getX(),
                pose.getY(),
                pose.getRotation().getDegrees());
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
                driveSubsystem.getGyroscopeRotation(),
                driveSubsystem.getModulePositions(),
                newPose);
    }

    public void setToCurrentVisionPose() {
        synchronized (this) {
            if (getVisionPose() != null) {
                setCurrentPose(getVisionPose());
            }
        }
    }

    private void setVisionPose(Pose2d visionPose2d) {
        synchronized (this) {
            this.visionPose2d = visionPose2d;
        }
    }

    private Pose2d getVisionPose() {
        synchronized (this) {
            return visionPose2d;
        }
    }

    /**
     * Resets the position on the field to 0,0 0-degrees, with forward being
     * downfield. This resets
     * what "forward" is for field oriented driving.
     */
    public void resetFieldPosition() {
        setCurrentPose(new Pose2d());
    }

    void apriltagVisionThreadProc() {
        AprilTagDetector detector = new AprilTagDetector();

        // look for tag36h11, correct 3 error bits
        detector.addFamily("tag36h11", 2);

        // Set up Pose Estimator - parameters are for a Microsoft Lifecam HD-3000
        // (https://www.chiefdelphi.com/t/wpilib-apriltagdetector-sample-code/421411/21)
        Config poseEstConfig = new AprilTagPoseEstimator.Config(
                VisionConstants.kTagSize, VisionConstants.kCameraVerticalFocalLength,
                VisionConstants.kCameraHorizontalFocalLength, VisionConstants.kCameraVerticalFocalCenter,
                VisionConstants.kCameraHorizontalFocalCenter);
        AprilTagPoseEstimator estimator = new AprilTagPoseEstimator(poseEstConfig);

        // Get the UsbCamera from CameraServer
        UsbCamera camera = CameraServer.startAutomaticCapture();
        // Set the resolution
        camera.setResolution(640, 480);

        // Get a CvSink. This will capture Mats from the camera
        CvSink cvSink = CameraServer.getVideo();
        // Setup a CvSource. This will send images back to the Dashboard
        CvSource outputStream = CameraServer.putVideo("Detected", 640, 480);

        // Mats are very memory expensive. Lets reuse these.
        Mat mat = new Mat();
        Mat grayMat = new Mat();

        // Instantiate once
        ArrayList<Long> tags = new ArrayList<>();
        Scalar outlineColor = new Scalar(0, 255, 0);
        Scalar crossColor = new Scalar(0, 0, 255);

        // This cannot be 'true'. The program will never exit if it is. This
        // lets the robot stop this thread when restarting robot code or
        // deploying.
        while (!Thread.interrupted()) {
            // Tell the CvSink to grab a frame from the camera and put it
            // in the source mat. If there is an error notify the output.
            if (cvSink.grabFrame(mat) == 0) {
                // Send the output the error.
                outputStream.notifyError(cvSink.getError());
                // skip the rest of the current iteration
                continue;
            }

            double timestamp = Timer.getFPGATimestamp();

            Imgproc.cvtColor(mat, grayMat, Imgproc.COLOR_RGB2GRAY);

            AprilTagDetection[] detections = detector.detect(grayMat);

            if (detections.length > 0) {
                softwareTab.add("Vision targets", true);
                softwareTab.add("targets", detections);
            } else {
                softwareTab.add("Vision targets", false);
            }

            // have not seen any tags yet
            for (AprilTagDetection detection : detections) {

                Pose3d tagInFieldFrame; // pose from WPILib resource

                if (aprilTagFieldLayout.getTagPose(detection.getId()).isPresent()
                        && detection.getDecisionMargin() > 50.) // margin < 20 seems bad; margin > 120 are good
                {
                    tagInFieldFrame = aprilTagFieldLayout.getTagPose(detection.getId()).get();
                } else {
                    System.out.println("bad id " + detection.getId() + " " + detection.getDecisionMargin());
                    continue;
                }

                // determine pose
                Transform3d pose = estimator.estimate(detection);

                drawFrustumOnMat(detection, mat, outlineColor, crossColor, pose);

                // These transformations are required for the correct robot pose.
                // They arise from the tag facing the camera thus Pi radians rotated or CCW/CW
                // flipped from the
                // mathematically described pose from the estimator that's what our eyes see.
                // The true rotation
                // has to be used to get the right robot pose.
                pose = new Transform3d(
                        new Translation3d(
                                pose.getX(),
                                pose.getY(),
                                pose.getZ()),
                        new Rotation3d(
                                -pose.getRotation().getX() - Math.PI,
                                -pose.getRotation().getY(),
                                pose.getRotation().getZ() - Math.PI));

                // OpenCV and WPILib estimator layout of axes is EDN and field WPILib is NWU;
                // need x -> -y , y -> -z , z -> x and same for differential rotations
                // pose = CoordinateSystem.convert(pose, CoordinateSystem.EDN(),
                // CoordinateSystem.NWU());
                // WPILib convert is wrong for transforms as of 2023.4.3 so use this patch for
                // now
                {
                    // corrected convert
                    CoordinateSystem from = CoordinateSystem.EDN();
                    CoordinateSystem to = CoordinateSystem.NWU();
                    pose = new Transform3d(
                            CoordinateSystem.convert(pose.getTranslation(), from, to),
                            CoordinateSystem.convert(new Rotation3d(), to, from)
                                    .plus(CoordinateSystem.convert(pose.getRotation(), from, to)));
                    // end of corrected convert
                }

                // transform to camera from robot chassis center at floor level
                Transform3d cameraInRobotFrame = new Transform3d(
                        VisionConstants.kCameraLocation,
                        VisionConstants.kCameraRotation);

                // robot in field is the composite of 3 pieces
                Pose3d robotInFieldFrame = ComputerVisionUtil.objectToRobotPose(tagInFieldFrame, pose,
                        cameraInRobotFrame);
                // end transforms to get the robot pose from this vision tag pose

                setVisionPose(robotInFieldFrame.toPose2d());

                double distance = visionPose2d.getTranslation().getDistance(this.getCurrentPose().getTranslation());
                if (Math.abs(distance) <= 1) {
                    if (VisionConstants.kUsingVision) {
                        poseEstimator.addVisionMeasurement(visionPose2d, timestamp);
                        SmartDashboard.putNumber("Last Image used", timestamp);
                    }
                }

            }
            outputStream.putFrame(mat);
        }

    }

    private static void drawFrustumOnMat(AprilTagDetection detection, Mat mat, Scalar outlineColor, Scalar crossColor,
            Transform3d pose) {
        // draw lines around the tag
        for (int i = 0; i <= 3; i++) {
            int j = (i + 1) % 4;
            Point pt1 = new Point(detection.getCornerX(i), detection.getCornerY(i));
            Point pt2 = new Point(detection.getCornerX(j), detection.getCornerY(j));
            Imgproc.line(mat, pt1, pt2, outlineColor, 2);
        }

        // mark the center of the tag
        double cx = detection.getCenterX();
        double cy = detection.getCenterY();
        int ll = 10;
        Imgproc.line(mat, new Point(cx - ll, cy), new Point(cx + ll, cy), crossColor, 2);
        Imgproc.line(mat, new Point(cx, cy - ll), new Point(cx, cy + ll), crossColor, 2);

        // identify the tag
        Imgproc.putText(
                mat,
                Integer.toString(detection.getId()),
                new Point(cx + ll, cy),
                Imgproc.FONT_HERSHEY_SIMPLEX,
                1,
                crossColor,
                3);

        { // draw a frustum in front of the AprilTag
          // use the estimated pose from above before any other transforms

            // camera same as above but different format for OpenCV
            float[] cameraParm = { (float) VisionConstants.kCameraVerticalFocalLength, 0.f,
                    (float) VisionConstants.kCameraVerticalFocalCenter,
                    0.f, (float) VisionConstants.kCameraHorizontalFocalLength,
                    (float) VisionConstants.kCameraHorizontalFocalCenter,
                    0.f, 0.f, 1.f };
            Mat K = new Mat(3, 3, CvType.CV_32F); // camera matrix
            K.put(0, 0, cameraParm);

            MatOfDouble distCoeffs = new MatOfDouble(); // not using any camera distortions so it's empty

            // 3D points of ideal, original corners, flat on the tag, scaled to the actual
            // tag size
            MatOfPoint3f bottom = new MatOfPoint3f(
                    new Point3(-1. * VisionConstants.kTagSize / 2., 1. * VisionConstants.kTagSize / 2., 0.),
                    new Point3(1. * VisionConstants.kTagSize / 2., 1. * VisionConstants.kTagSize / 2., 0.),
                    new Point3(1. * VisionConstants.kTagSize / 2., -1. * VisionConstants.kTagSize / 2., 0.),
                    new Point3(-1. * VisionConstants.kTagSize / 2., -1. * VisionConstants.kTagSize / 2., 0.));

            // 3D points of the ideal, original corners, in front of the tag to make a
            // frustum, scaled to the actual tag size
            // note that the orientation and size of the face of the box can be controlled
            // by the sign of the "Z"
            // value of the "top" variable.
            // "-" (negative) gives larger top facing straight away from the plane of the
            // tag
            // "+" (positive) gives smaller top facing toward the camera
            MatOfPoint3f top = new MatOfPoint3f(
                    new Point3(-1. * VisionConstants.kTagSize / 2., 1. * VisionConstants.kTagSize / 2.,
                            -0.7 * VisionConstants.kTagSize),
                    new Point3(1. * VisionConstants.kTagSize / 2., 1. * VisionConstants.kTagSize / 2.,
                            -0.7 * VisionConstants.kTagSize),
                    new Point3(1. * VisionConstants.kTagSize / 2., -1. * VisionConstants.kTagSize / 2.,
                            -0.7 * VisionConstants.kTagSize),
                    new Point3(-1. * VisionConstants.kTagSize / 2., -1. * VisionConstants.kTagSize / 2.,
                            -0.7 * VisionConstants.kTagSize));

            double[] rotationVector = pose.getRotation().getQuaternion().toRotationVector().getData(); // 3x1 3
                                                                                                       // rows 1
                                                                                                       // col

            Mat T = new Mat(3, 1, CvType.CV_64FC1);
            Mat R = new Mat(3, 1, CvType.CV_64FC1);
            T.put(0, 0, pose.getX(), pose.getY(), pose.getZ());
            R.put(0, 0, rotationVector[0], rotationVector[1], rotationVector[2]);

            MatOfPoint2f imagePointsBottom = new MatOfPoint2f();
            Calib3d.projectPoints(bottom, R, T, K, distCoeffs, imagePointsBottom);

            MatOfPoint2f imagePointsTop = new MatOfPoint2f();
            Calib3d.projectPoints(top, R, T, K, distCoeffs, imagePointsTop);

            ArrayList<Point> topCornerPoints = new ArrayList<Point>();

            // draw from bottom points to top points - pillars
            for (int i = 0; i < 4; i++) {
                double x1;
                double y1;
                double x2;
                double y2;
                x1 = imagePointsBottom.get(i, 0)[0];
                y1 = imagePointsBottom.get(i, 0)[1];
                x2 = imagePointsTop.get(i, 0)[0];
                y2 = imagePointsTop.get(i, 0)[1];

                topCornerPoints.add(new Point(x2, y2));

                Imgproc.line(mat,
                        new Point(x1, y1),
                        new Point(x2, y2),
                        outlineColor,
                        2);
            }

            MatOfPoint topCornersTemp = new MatOfPoint();
            topCornersTemp.fromList(topCornerPoints);

            ArrayList<MatOfPoint> topCorners = new ArrayList<MatOfPoint>();
            topCorners.add(topCornersTemp);

            Imgproc.polylines(mat, topCorners, true, outlineColor, 2);
        } /* end draw a frustum in front of the AprilTag */
    }

}
