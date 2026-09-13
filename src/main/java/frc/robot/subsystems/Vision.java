package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Vision {
    
    PhotonCamera frontLeft = new PhotonCamera("frontLeft");
    PhotonCamera frontRight = new PhotonCamera("frontRight");
    PhotonCamera backLeft = new PhotonCamera("backLeft");
    PhotonCamera backRight = new PhotonCamera("backRight");

    PhotonCamera[] cameras = {frontLeft, frontRight, backLeft, backRight};
    
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
