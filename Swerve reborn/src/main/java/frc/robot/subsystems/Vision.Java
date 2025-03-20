package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.TunerConstants;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.rmi.server.RMIClassLoader;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
    private final PhotonCamera camera = new PhotonCamera("FHD_Camera");
    private static final double TARGET_DISTANCE_METERS = 1.0; // Stop at 1 meter from the tag
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    private final CommandSwerveDrivetrain drivetrain;
    private final SwerveRequest.FieldCentricFacingAngle snapTo = new SwerveRequest.FieldCentricFacingAngle()
        .withDeadband(MaxSpeed*0.035)
        .withDriveRequestType(DriveRequestType.Velocity);
    
    
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric();
            //.withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate*0.1)
            //.withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    public Vision(CommandSwerveDrivetrain train) {
        System.out.println("Vision System Initialized");
        
        // Print values from methods correctly
        System.out.println("Target Yaw: " + getTargetYaw());
        System.out.println("Target Distance: " + getTargetDistance());
        drivetrain = train;
    }

    public PhotonTrackedTarget getBestTarget() {
        PhotonPipelineResult result = camera.getLatestResult();
        if (result.hasTargets()) {
            return result.getBestTarget();
        }
        return null;
    }

    public Transform3d getTargetTransform() {
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

    // public Command followAprilTag(){
        
    //         var target = this.getBestTarget();
        
    //         if (target != null) {
    //             // Get distance and angle to the AprilTag
    //             double targetYaw = this.getTargetYaw(); // Degrees
    //             double targetDistance = this.getTargetDistance(); // Meters

    //             System.out.println("Yaw: " + targetYaw);
    //             System.out.println("Target Distance: " + targetDistance);

    //             System.out.println("Pose: " + drivetrain.getState().Pose);

    //             // Compute movement speeds
    //             double forwardSpeed =  -Math.max(0.2, Math.min(1.0, (targetDistance - TARGET_DISTANCE_METERS) * MaxSpeed)); // Speed scales based on distance
    //             double rotationSpeed = -targetYaw * MaxAngularRate * 0.2; // Scale yaw to rotation (negative to correct direction)
    //             System.out.println("Speed: " + 0);
    //             // Create a movement request
    //             return drivetrain.applyRequest(() ->
    //                 snapTo.withVelocityX(0)
    //                 .withVelocityY(0)
    //                 .withTargetDirection(Rotation2d.fromDegrees(180))
    //             );
    //         } else {
    //             return drivetrain.applyRequest(() -> 
    //                 drive.withVelocityX(0)
    //                 .withVelocityY(0)
    //                 .withRotationalRate(0)
    //             );
    //         }
    // }

    public Command followAprilTag() {
        return run(() -> {
            var target = this.getBestTarget();
            if (target != null) {
                double targetYaw = this.getTargetYaw(); // Degrees
                double targetDistance = this.getTargetDistance(); // Meters
    
                System.out.println("Yaw: " + targetYaw);
                System.out.println("Distance: " + targetDistance);
    
                if (targetDistance <= TARGET_DISTANCE_METERS) {
                    System.out.println("Arrived at target!");
                    drivetrain.applyRequest(() -> 
                        drive.withVelocityX(0) // Stop moving
                             .withVelocityY(0)
                             .withRotationalRate(0)
                    );
                    return;
                }
    
                // Compute forward speed based on distance, limit between 0.2 and 1.0
                double forwardSpeed = -Math.max(0.2, Math.min(1.0, (targetDistance - TARGET_DISTANCE_METERS) * MaxSpeed));
    
                System.out.println("Speed: " + forwardSpeed);
    
                drivetrain.applyRequest(() -> 
                    drive.withVelocityX(forwardSpeed) // Move forward
                         .withVelocityY(0)
                         .withRotationalRate(0)
                );
            } else {
                System.out.println("Tag not found");
            }
        }).withTimeout(5); // Safety timeout to prevent endless running
    }

    public Command PayGorn(){
        return run(() -> {
            System.out.println("PayGorn");
        });
    }
}