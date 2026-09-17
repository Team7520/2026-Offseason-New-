package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.List;
import java.util.Optional;

import org.junit.jupiter.api.Test;
import org.photonvision.targeting.PhotonPipelineMetadata;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

class VisionTest {
    private final AprilTagFieldLayout fieldLayout = new AprilTagFieldLayout(
        List.of(new AprilTag(1, new Pose3d(10, 5, 1, new Rotation3d()))), 16, 8);
    private final Transform3d robotToCamera = new Transform3d(0.5, -0.25, 1, new Rotation3d());

    @Test
    void processesValidFramesAroundEmptyAndUnknownTagFramesWithTheirOwnTimestamps() {
        var measurements = Vision.estimateRobotFieldMeasurements(List.of(
            frame(1_100_000, 1, 3, 1),
            emptyFrame(1_200_000),
            frame(1_300_000, 99, 3, 1),
            frame(1_400_000, 1, 2, 0.5)), fieldLayout, robotToCamera);

        assertEquals(2, measurements.size());
        assertEquals(1.1, measurements.get(0).timestampSeconds(), 1e-9);
        assertEquals(6.5, measurements.get(0).pose().getX(), 1e-9);
        assertEquals(4.25, measurements.get(0).pose().getY(), 1e-9);
        assertEquals(1.4, measurements.get(1).timestampSeconds(), 1e-9);
        assertEquals(7.5, measurements.get(1).pose().getX(), 1e-9);
        assertEquals(4.75, measurements.get(1).pose().getY(), 1e-9);
    }

    @Test
    void retainsEarlierValidFrameWhenNewestFrameHasNoTargets() {
        var measurements = Vision.estimateRobotFieldMeasurements(List.of(
            frame(2_000_000, 1, 3, 1), emptyFrame(2_100_000)), fieldLayout, robotToCamera);

        assertEquals(1, measurements.size());
        assertEquals(2.0, measurements.get(0).timestampSeconds(), 1e-9);
        assertEquals(6.5, measurements.get(0).pose().getX(), 1e-9);
        assertEquals(4.25, measurements.get(0).pose().getY(), 1e-9);
    }

    private static PhotonPipelineResult frame(long captureMicros, int tagId, double x, double y) {
        PhotonTrackedTarget target = new PhotonTrackedTarget();
        target.fiducialId = tagId;
        target.bestCameraToTarget = new Transform3d(x, y, 0, new Rotation3d());
        return new PhotonPipelineResult(
            new PhotonPipelineMetadata(captureMicros, captureMicros + 10_000, 1, 0),
            List.of(target), Optional.empty());
    }

    private static PhotonPipelineResult emptyFrame(long captureMicros) {
        return new PhotonPipelineResult(
            new PhotonPipelineMetadata(captureMicros, captureMicros + 10_000, 1, 0),
            List.of(), Optional.empty());
    }
}
