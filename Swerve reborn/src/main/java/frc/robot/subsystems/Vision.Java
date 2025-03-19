package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
    private final PhotonCamera camera1 = new PhotonCamera("FHD_Camera");

    public VisionSubsystem() {}

    public PhotonTrackedTarget getBestTarget() {
        PhotonPipelineResult result = camera.getLatestResult();
        if (result.hasTargets()) {
            return result.getBestTarget();
        }
        return null;
    }

    public Transform2d getTargetTransform() {
        PhotonTrackedTarget target = getBestTarget();
        if (target != null) {
            return target.getBestCameraToTarget();
        }
        return null;
    }
}