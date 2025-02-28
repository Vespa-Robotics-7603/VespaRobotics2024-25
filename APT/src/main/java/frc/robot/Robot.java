package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.geometry.Transform3d;

public class Robot extends TimedRobot {

    private PhotonCamera camera1; // Declare the camera instance
    private PhotonCamera camera2; //Declare Camera 2

    @Override
    public void robotInit() {
        // Initialize PhotonCamera
        camera1 = new PhotonCamera("FHD_Camera1");
        camera2 = new PhotonCamera("FHD_Camera2");
        
        // Start USB camera streaming (optional)
        UsbCamera usbCamera1 = CameraServer.startAutomaticCapture(0);
        usbCamera1.setResolution(352, 288);

        UsbCamera usbCamera2 = CameraServer.startAutomaticCapture(1);
        usbCamera2.setResolution(352, 288);
    }

    @Override
    public void robotPeriodic() {
        processCamera(camera1, "Camera 1");
        processCamera(camera2, "Camera 2");
    }

    private void processCamera(PhotonCamera camera, String cameraName) {
        PhotonPipelineResult result = camera.getLatestResult();
        if (result.hasTargets()) {
            PhotonTrackedTarget target = result.getBestTarget();
            double yaw = target.getYaw();
            Transform3d camToTarget = target.getBestCameraToTarget();

            if (camToTarget != null) {
                double distance = Math.sqrt(
                    camToTarget.getTranslation().getX() * camToTarget.getTranslation().getX() +
                    camToTarget.getTranslation().getY() * camToTarget.getTranslation().getY() +
                    camToTarget.getTranslation().getZ() * camToTarget.getTranslation().getZ()
                );

                int AprilId = target.getFiducialId();

                // Print data per camera
                System.out.println("[" + cameraName + "] Camera to Target: " + camToTarget);
                System.out.println("[" + cameraName + "] Distance: " + distance);
                System.out.println("[" + cameraName + "] Detected ID: " + AprilId);
                System.out.println("[" + cameraName + "] Yaw: " + yaw);
            }
        } else {
            System.out.println("[" + cameraName + "] No targets detected.");
        }
    }
}
