package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.geometry.Transform3d;

public class Robot extends TimedRobot {

    private PhotonCamera camera; // Declare the camera instance

    @Override
    public double getPeriod(){
        return 0.02; // run every 2ms
    }
    
    @Override
    public void robotInit() {
        // Initialize PhotonCamera
        camera = new PhotonCamera("FHD_Camera");
        
        // Start camera streaming (optional for visualization)
        UsbCamera usbCamera = CameraServer.startAutomaticCapture();
        usbCamera.setResolution(352, 288);
    }

    @Override
    public void robotPeriodic() {
        // Get the latest result from the PhotonCamera
        PhotonPipelineResult result = camera.getLatestResult();
        // Check if the camera has detected any targets
        if (result.hasTargets()) {
            // Get the best target
            PhotonTrackedTarget target = result.getBestTarget();

            // Retrieve target information
            double yaw = target.getYaw(); // Horizontal angle to target
            //double pitch = target.getPitch(); // Vertical angle to target

            // Get the transform from the camera to the target (if available)
            Transform3d camToTarget = target.getBestCameraToTarget();
            if (camToTarget != null) {
                System.out.println("Camera to Target Transform: " + camToTarget);

                // Calculate the 3D distance from the camera to the target
                double distance = Math.sqrt(
                    camToTarget.getTranslation().getX() * camToTarget.getTranslation().getX() +
                    camToTarget.getTranslation().getY() * camToTarget.getTranslation().getY() +
                    camToTarget.getTranslation().getZ() * camToTarget.getTranslation().getZ()
                );

                // Print the distance to the console
                System.out.println("Distance to Target: " + distance);
                // Get the id of the apriltag
                int AprilId = target.getFiducialId();
                // Print apriltag id
                System.out.println("Detected ID: " + AprilId);
                // Debug output for yaw and pitch
                System.out.println("Yaw: " + yaw);
                //System.out.println( "Pitch: " + pitch);
            }
        } else {
            // No targets detected
            System.out.println("No targets detected.");
        }
    }
}
