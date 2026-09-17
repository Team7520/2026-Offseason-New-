package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;

public class Vision {

    AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

    // CAD +X is robot right, +Y is forward, and +Z is up (millimeters).
    // WPILib +X is forward, +Y is left, and +Z is up (meters).
    // Cameras are upside down; negative WPILib pitch points the lens upward.
    private final Transform3d robotToFrontLeft = new Transform3d(
        new Translation3d(0.283304215, -0.215717783, 0.536052163),
        new Rotation3d(Units.degreesToRadians(180), Units.degreesToRadians(-24.117007), Units.degreesToRadians(45))
    );

    private final Transform3d robotToFrontRight = new Transform3d(
        new Translation3d(0.283311550, -0.346804215, 0.536052163),
        new Rotation3d(Units.degreesToRadians(180), Units.degreesToRadians(-24.117007), Units.degreesToRadians(-45))
    );

    private final Transform3d robotToBackLeft = new Transform3d(
        new Translation3d(0.170198349, 0.322260216, 0.536052163),
        new Rotation3d(Units.degreesToRadians(180), Units.degreesToRadians(-24.117007), Units.degreesToRadians(135))
    );

    private final Transform3d robotToBackRight = new Transform3d(
        new Translation3d(0.211995785, -0.289661550, 0.536052163),
        new Rotation3d(Units.degreesToRadians(180), Units.degreesToRadians(-24.117007), Units.degreesToRadians(-135))
    );

    Transform3d[] robotToCameras = {robotToFrontLeft, robotToFrontRight, robotToBackLeft, robotToBackRight};

    PhotonCamera frontLeft = new PhotonCamera("frontLeft");
    PhotonCamera frontRight = new PhotonCamera("frontRight");
    PhotonCamera backLeft = new PhotonCamera("backLeft");
    PhotonCamera backRight = new PhotonCamera("backRight");

    PhotonCamera[] cameras = {frontLeft, frontRight, backLeft, backRight};
    
    private final PhotonPoseEstimator frontLeftEstimator = new PhotonPoseEstimator(
        fieldLayout,
        robotToFrontLeft
    );

    private final PhotonPoseEstimator frontRightEstimator = new PhotonPoseEstimator(
        fieldLayout,
        robotToFrontRight
    );

    private final PhotonPoseEstimator backLeftEstimator = new PhotonPoseEstimator(
        fieldLayout,
        robotToBackLeft
    );

    private final PhotonPoseEstimator backRightEstimator = new PhotonPoseEstimator(
        fieldLayout,
        robotToBackRight
    );

    private final PhotonPoseEstimator[] estimators = {
        frontLeftEstimator, frontRightEstimator, backLeftEstimator, backRightEstimator
    };

    public List<EstimatedRobotPose> getTargets(){
        List<EstimatedRobotPose> visionEsts = new ArrayList<>();
        
        for (int i = 0; i < cameras.length; i++) {
            PhotonCamera camera = cameras[i];
            PhotonPoseEstimator estimator = estimators[i];
            
            for (var result : camera.getAllUnreadResults()) {
                Optional<EstimatedRobotPose> est = estimator.estimateCoprocMultiTagPose(result);
                if (est.isEmpty()) {
                    est = estimator.estimateLowestAmbiguityPose(result);
                }
                est.ifPresent(visionEsts::add);
            }
        }
        return visionEsts;
    }

    public record VisionMeasurement(Pose2d pose, double timestampSeconds) {}

    public List<VisionMeasurement> getCurrentRobotFieldMeasurements(int index) {
        return estimateRobotFieldMeasurements(
            cameras[index].getAllUnreadResults(), fieldLayout, robotToCameras[index]);
    }

    static List<VisionMeasurement> estimateRobotFieldMeasurements(
            List<PhotonPipelineResult> results, AprilTagFieldLayout fieldLayout, Transform3d robotToCamera) {
        List<VisionMeasurement> measurements = new ArrayList<>();
        for (PhotonPipelineResult result : results) {
            if (!result.hasTargets()) {
                continue;
            }
            PhotonTrackedTarget target = result.getBestTarget();
            Optional<Pose3d> tagPose = fieldLayout.getTagPose(target.getFiducialId());
            if (tagPose.isEmpty()) {
                continue;
            }
            Pose3d robotPose =
                PhotonUtils.estimateFieldToRobotAprilTag(
                    target.getBestCameraToTarget(),
                    tagPose.get(),
                    robotToCamera.inverse());
            // Keep the capture timestamp paired with the frame used to estimate the pose.
            measurements.add(new VisionMeasurement(robotPose.toPose2d(), result.getTimestampSeconds()));
        }
        return measurements;
    }

    // public void GoToAzimuth2() {
    //     boolean targetVisible = false;
    //     double targetYaw = 0.0;
    //     var results = backRight.getAllUnreadResults();
    //     if (!results.isEmpty()) {
    //         // Camera processed a new frame since last
    //         // Get the last one in the list.
    //         var result = results.get(results.size() - 1);
    //         if (result.hasTargets()) {
    //             // At least one AprilTag was seen by the camera
    //             for (var target : result.getTargets()) {
    //                 if (target.getFiducialId() == 7) {
    //                     // Found Tag 7, record its information
    //                     targetYaw = target.getYaw();
    //                     targetVisible = true;
    //                 }
    //             }
    //         }
    //     }
    // }
}
