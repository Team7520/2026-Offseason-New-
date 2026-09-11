package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;

public class Vision {
    private List<PhotonPoseEstimator> estimators = null;

    AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

    private final Transform3d robotToFrontLeft = new Transform3d(
        new Translation3d(-0.3217183558, 0.1713393322, 0.5360775664), 
        new Rotation3d(0, Units.degreesToRadians(-23), Units.degreesToRadians(45))
    );

    private final Transform3d robotToFrontRight = new Transform3d(
        new Translation3d(0.288096706, 0.2131367848, 0.5360775664), 
        new Rotation3d(0, Units.degreesToRadians(-23), Units.degreesToRadians(315))
    );

    private final Transform3d robotToBackLeft = new Transform3d(
        new Translation3d(0.2162596386, 0.2823385006, 0.5360775664), 
        new Rotation3d(0, Units.degreesToRadians(-23), Units.degreesToRadians(135))
    );

    private final Transform3d robotToBackRight = new Transform3d(
        new Translation3d(0.3452393654, 0.2823458412, 0.5360775664), 
        new Rotation3d(0, Units.degreesToRadians(-23), Units.degreesToRadians(225))
    );

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

    public List<PhotonTrackedTarget> getTargets(){
        List<PhotonTrackedTarget> targets = new ArrayList<>();
        for (PhotonCamera camera : cameras) {
            PhotonPipelineResult result = camera.getLatestResult();
            if (result.hasTargets()) {
                for (PhotonTrackedTarget target : result.getTargets()) {
                    targets.add(target);
                }
            }
        }

        return targets;        
    }
}