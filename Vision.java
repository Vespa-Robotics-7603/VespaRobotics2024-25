package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
    private final PhotonCamera camera = new PhotonCamera("FHD_Camera");

    public Vision() {

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

        public double getTargetYaw() {
            PhotonTrackedTarget target = getBestTarget();
            return (target != null) ? target.getYaw() : 0.0;
        }

        public double getTargetDistance() {
            PhotonTrackedTarget target = getBestTarget();
            return (target != null) ? target.getBestCameraToTarget().getX() : -1;
        }
    }
}
