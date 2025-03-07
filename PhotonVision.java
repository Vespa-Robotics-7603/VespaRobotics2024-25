package frc.robot.subsystems;

import edu.wpi.first.wpilibj.TimedRobot;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.geometry.Transform3d;

import java.util.List;
import java.util.ArrayList;

public class PhotonVision implements Subsystem {

    public class ph_v_target {
        public double distance;
        public int aprilid;
        public Transform3d camtotarget;
        public double yaw;
    }

    private PhotonCamera camera1; // Declare the Camera1 instance
    private PhotonCamera camera2; //Declare Camera2

    public List<ph_v_target> targets = new ArrayList<ph_v_target>();

    public ph_v_target get_target_with_april_id(int aprilid) {

        for (ph_v_target target : targets) {
            
            if (target.aprilid == aprilid) {

                return target;

            }

        }

        return null;

    }

    @Override
    public void periodic() {
        // Initialize PhotonCamera
        camera1 = new PhotonCamera("FHD_Camera");
        camera2 = new PhotonCamera("Cam_2");
        
        // Start USB camera streaming (optional)
        UsbCamera usbCamera1 = CameraServer.startAutomaticCapture(0);
        usbCamera1.setResolution(352, 288);

        UsbCamera usbCamera2 = CameraServer.startAutomaticCapture(1);
        usbCamera2.setResolution(352, 288);
    }


    public void AprilRun() {
        processCamera(camera1, "Camera 1");
        processCamera(camera2, "Camera 2");
    }

    private void processCamera(PhotonCamera camera, String cameraName) {
        PhotonPipelineResult result = camera.getLatestResult();
        
        if (result.hasTargets()) {

            targets.clear();

            for (PhotonTrackedTarget target : result.targets) {

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

                    ph_v_target new_target = new ph_v_target();

                    new_target.distance = distance;
                    new_target.yaw = yaw;
                    new_target.aprilid = AprilId;
                    new_target.camtotarget = camToTarget;

                    targets.add(new_target);

                }

            }

        } 
        
        else {
            System.out.println("[" + cameraName + "] No targets detected.");
        }
    }
}
