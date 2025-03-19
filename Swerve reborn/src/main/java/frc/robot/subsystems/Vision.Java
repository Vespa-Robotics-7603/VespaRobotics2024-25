package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision implements Subsystem {
    private final PhotonCamera camera1 = new PhotonCamera("FHD_Camera");

    //Note: 
    //UsbCamera usbCamera1 = CameraServer.startAutomaticCapture(0);
    //usbCamera1.setResolution(352, 288);
    // This is maybe not needed

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