package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
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

    private final Transform3d robotToFrontLeft = new Transform3d(
        new Translation3d(-0.3217183558,0.1713393322,0.5360775664),
        new Rotation3d(Units.degreesToRadians(180), Units.degreesToRadians(-23), Units.degreesToRadians(-45))
    );

    private final Transform3d robotToFrontRight = new Transform3d(
        new Translation3d(0.288096706, 0.2131367848, 0.5360775664),
        new Rotation3d(Units.degreesToRadians(180), Units.degreesToRadians(-23), Units.degreesToRadians(45))
    );

    private final Transform3d robotToBackLeft = new Transform3d(
        new Translation3d(.2162596386, .2823385006, .5360775664),
        new Rotation3d(Units.degreesToRadians(180), Units.degreesToRadians(-23), Units.degreesToRadians(-135))
    );

    private final Transform3d robotToBackRight = new Transform3d(
        new Translation3d(.3452393654, .2823458412, .5360775664),
        new Rotation3d(Units.degreesToRadians(180), Units.degreesToRadians(-23), Units.degreesToRadians(135))
    );

    Transform3d[] robotToCameras = {robotToFrontLeft, robotToFrontRight, robotToBackLeft, robotToBackRight};

    PhotonCamera frontLeft = new PhotonCamera("frontLeft");
    PhotonCamera frontRight = new PhotonCamera("frontRight");
    PhotonCamera backLeft = new PhotonCamera("backLeft");
    PhotonCamera backRight = new PhotonCamera("backRight");

    PhotonCamera[] cameras = {frontLeft, frontRight, backLeft, backRight};
    
    private final PhotonPoseEstimator frontLeftEstimator = new PhotonPoseEstimator(
        fieldLayout,
        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        robotToFrontLeft
    );

    private final PhotonPoseEstimator frontRightEstimator = new PhotonPoseEstimator(
        fieldLayout,
        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        robotToFrontRight
    );

    private final PhotonPoseEstimator backLeftEstimator = new PhotonPoseEstimator(
        fieldLayout,
        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        robotToBackLeft
    );

    private final PhotonPoseEstimator backRightEstimator = new PhotonPoseEstimator(
        fieldLayout,
        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
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

    public double getCaptureTime(int index) {
        PhotonPipelineResult result;
        result = cameras[index].getLatestResult();
        return result.getTimestampSeconds();
  }

    private PhotonPipelineResult getLatestCameraResult(PhotonCamera camera) {
        List<PhotonPipelineResult> results = camera.getAllUnreadResults();
        if (results.isEmpty()) {
            return new PhotonPipelineResult();
        }
        return results.get(results.size() - 1);
    }

    public Pose2d getCurrentRobotFieldPose(int index) {
        PhotonPipelineResult result = null;
        result = getLatestCameraResult(cameras[index]);
        Transform3d robotToCamera = robotToCameras[index];
        PhotonTrackedTarget target = result.getBestTarget();
        if (target != null) {
            Pose3d robotPose =
                PhotonUtils.estimateFieldToRobotAprilTag(
                target.getBestCameraToTarget(),
                fieldLayout.getTagPose(target.getFiducialId()).get(),
                robotToCamera.inverse());
            return robotPose.toPose2d();
        } else {
            return null;
        }
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