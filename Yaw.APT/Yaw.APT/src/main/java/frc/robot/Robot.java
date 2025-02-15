package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.geometry.Transform3d;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

class driveTrain{
  // initialzing vars
  VictorSPX LeftMotor1;
  VictorSPX LeftMotor2;

  VictorSPX RightMotor1;
  VictorSPX RightMotor2;
  //ControlMode1 ControlMode;

  // constructor
  public driveTrain(VictorSPX LM1, VictorSPX LM2, VictorSPX RM1, VictorSPX RM2){
      LeftMotor1 = LM1;
      LeftMotor2 = LM2;

      RightMotor1 = RM1;
      RightMotor2 = RM2;
      //control = new ControlMode1();
  }
}

public class Robot extends TimedRobot {
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  VictorSPX motorL2 = new VictorSPX(7);
  VictorSPX motorR1 = new VictorSPX(5);
  VictorSPX motorR2 = new VictorSPX(6);
  VictorSPX motorL1 = new VictorSPX(10);
  driveTrain drive = new driveTrain(motorL1, motorL2, motorR1, motorR2);

    private PhotonCamera camera; // Declare the camera instance

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
                int aprilid = target.getFiducialId();
                // Print apriltag id
                System.out.println("Detected ID: " + aprilid);
                // Debug output for yaw and pitch
                System.out.println("Yaw: " + yaw);
                //System.out.println( "Pitch: " + pitch);
            }
        } else {
            // No targets detected
            System.out.println("No targets detected.");
        }
    }

    @Override
    public void autonomousPeriodic() {
        // Get the latest result from the PhotonCamera
        PhotonPipelineResult result = camera.getLatestResult();

        // Check if there is a target
        if (result.hasTargets()) {
            // Get the best target
            PhotonTrackedTarget target = result.getBestTarget();
            double yaw = target.getYaw(); // Get the horizontal angle to the target

            // Proportional control factor (adjust as needed)
            double kP = 0.02;
            double turnSpeed = kP * yaw; 

            // Limit the turn speed to avoid excessive motion
            turnSpeed = Math.max(-0.5, Math.min(0.5, turnSpeed)); 

            // Apply turning (positive yaw -> turn right, negative yaw -> turn left)
            motorL1.set(ControlMode.PercentOutput, turnSpeed);
            motorL2.set(ControlMode.PercentOutput, turnSpeed);
            motorR1.set(ControlMode.PercentOutput, -turnSpeed);
            motorR2.set(ControlMode.PercentOutput, -turnSpeed);

            // Debug output
            System.out.println("Turning towards target. Yaw: " + yaw + " | Speed: " + turnSpeed);
        } else {
            // Stop motors if no target is detected
            motorL1.set(ControlMode.PercentOutput, 0);
            motorL2.set(ControlMode.PercentOutput, 0);
            motorR1.set(ControlMode.PercentOutput, 0);
            motorR2.set(ControlMode.PercentOutput, 0);

            System.out.println("No targets detected. Stopping.");
        }
    }
}
