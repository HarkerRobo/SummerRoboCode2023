package frc.robot.util;

import java.util.ArrayList;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class CameraPoseEstimation {
    private static CameraPoseEstimation instance;

    private AprilTagFieldLayout aprilTagFieldLayout;
    private PhotonCamera camera;

    Transform3d robotToCamera;
    PhotonPoseEstimator robotPoseEstimator;

    /**
     * Define all april tags used;
     */

    public static final List<AprilTag> APRIL_TAGS = List.of(
            new AprilTag(
                    1,
                    new Pose3d(
                            Units.inchesToMeters(610.77),
                            Units.inchesToMeters(42.19),
                            Units.inchesToMeters(18.22),
                            new Rotation3d(0.0, 0.0, Math.PI))),
            new AprilTag(
                    2,
                    new Pose3d(
                            Units.inchesToMeters(610.77),
                            Units.inchesToMeters(108.19),
                            Units.inchesToMeters(18.22),
                            new Rotation3d(0.0, 0.0, Math.PI))),
            new AprilTag(
                    3,
                    new Pose3d(
                            Units.inchesToMeters(610.77),
                            Units.inchesToMeters(174.19), // FIRST's diagram has a typo (it says 147.19)
                            Units.inchesToMeters(18.22),
                            new Rotation3d(0.0, 0.0, Math.PI))),
            new AprilTag(
                    4,
                    new Pose3d(
                            Units.inchesToMeters(636.96),
                            Units.inchesToMeters(265.74),
                            Units.inchesToMeters(27.38),
                            new Rotation3d(0.0, 0.0, Math.PI))),
            new AprilTag(
                    5,
                    new Pose3d(
                            Units.inchesToMeters(14.25),
                            Units.inchesToMeters(265.74),
                            Units.inchesToMeters(27.38),
                            new Rotation3d())),
            new AprilTag(
                    6,
                    new Pose3d(
                            Units.inchesToMeters(40.45),
                            Units.inchesToMeters(174.19), // FIRST's diagram has a typo (it says 147.19)
                            Units.inchesToMeters(18.22),
                            new Rotation3d())),
            new AprilTag(
                    7,
                    new Pose3d(
                            Units.inchesToMeters(40.45),
                            Units.inchesToMeters(108.19),
                            Units.inchesToMeters(18.22),
                            new Rotation3d())),
            new AprilTag(
                    8,
                    new Pose3d(
                            Units.inchesToMeters(40.45),
                            Units.inchesToMeters(42.19),
                            Units.inchesToMeters(18.22),
                            new Rotation3d())));

    public CameraPoseEstimation() {
        aprilTagFieldLayout = new AprilTagFieldLayout(CameraPoseEstimation.APRIL_TAGS, 16.4846, 8.1026);
        // define the camera name as "limelight"
        camera = new PhotonCamera("limelight");
        
        // define loocation of the camera based on the robot
        robotToCamera = new Transform3d(
                new Translation3d(0, Units.inchesToMeters(10.81259), 0), new Rotation3d(0, 0, 0));
        var cameraList = new ArrayList<Pair<PhotonCamera, Transform3d>>();
        cameraList.add(new Pair<PhotonCamera, Transform3d>(camera, robotToCamera));
        robotPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, camera, robotToCamera);
    }

    public PhotonCamera getCamera() {
        return camera;
    }

    public static CameraPoseEstimation getInstance() {
        if (instance == null) {
            instance = new CameraPoseEstimation();
        }
        return instance;
    }
}
