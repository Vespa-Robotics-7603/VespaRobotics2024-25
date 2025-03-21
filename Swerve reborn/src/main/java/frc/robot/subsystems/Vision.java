package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.TunerConstants;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
    private final PhotonCamera camera;
    private static final double TARGET_DISTANCE_METERS = 1.0; // Stop at 1 meter from the tag
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    private final CommandSwerveDrivetrain drivetrain;
    private final SwerveRequest.FieldCentricFacingAngle snapTo = new SwerveRequest
        .FieldCentricFacingAngle()
        .withDeadband(MaxSpeed*0.035)
        .withDriveRequestType(DriveRequestType.Velocity);

    
    private final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric();
            //.withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate*0.1)
            //.withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    public Vision(CommandSwerveDrivetrain train, String Cam_Var) {
        System.out.println("Vision System Initialized");
        
        this.camera = new PhotonCamera(Cam_Var);

        // Print values from methods correctly
        System.out.println("Target Yaw: " + getTargetYaw());
        System.out.println("Target Distance: " + getTargetDistance());
        drivetrain = train;

        snapTo.HeadingController = new PhoenixPIDController(1, 0, 0);
        snapTo.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
        turnPID.enableContinuousInput(-Math.PI, Math.PI);
        tmr.restart();
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
    public double getTargetDistanceY() {
        PhotonTrackedTarget target = getBestTarget();
        return (target != null) ? target.getBestCameraToTarget().getY() : -1;
    }

    PhoenixPIDController turnPID = new PhoenixPIDController(0, 0, 0);
    PhoenixPIDController drivePID = new PhoenixPIDController(0, 0, 0);
    Timer tmr = new Timer();
    public void followAprilTag(){
        var target = this.getBestTarget();
    
        if (target != null) {
            double distance2tag = this.getTargetDistance(); // distance to tag in meters???
            double targetYaw = this.getTargetYaw(); // Degrees
            double xPose = this.getTargetDistance(); // Meters
            double yPose = this.getTargetDistanceY(); // IDK Maybe meters
            double CurrentRobotX = drivetrain.getState().Pose.getX();
            double CurrentRobotYaw = drivetrain.getState().Pose.getRotation().getRadians();

            double targetSetPoint = CurrentRobotX + xPose - 1; //Where the robot should go
            double targetSetPointYaw = CurrentRobotYaw + targetYaw - 0;


            double drivething = drivePID.calculate(CurrentRobotX,targetSetPoint,tmr.get());
            double turnthing = turnPID.calculate(CurrentRobotYaw,targetSetPointYaw,tmr.get());
            // Current Time Stamp Never 0!!!!!!!!!!!!!

            System.out.println("Yaw: " + targetYaw);
            System.out.println("(X) Target Distance: " + xPose);
            System.out.println("Pose: " + drivetrain.getState().Pose);
            System.out.println("Y:" + yPose);

            System.out.println(drivething);
            System.out.println(turnthing);
            System.out.println("turn cur yaw" + CurrentRobotYaw);
            System.out.println("target yaw" + targetSetPointYaw);

            // Compute movement speeds
            //double forwardSpeed =  -Math.max(0.2, Math.min(1.0, (xPose - TARGET_DISTANCE_METERS) * MaxSpeed)); // Speed scales based on distance
            //double rotationSpeed = -targetYaw * MaxAngularRate * 0.2; // Scale yaw to rotation (negative to correct direction)

            drivetrain.setControl(
                drive
                    .withVelocityX(distance2tag/5)
                    .withVelocityY(0)
                    .withRotationalRate(yPose)
            );

        } else {
            // return run(() -> {
            //     System.out.println("No April tag!");
            // });
            System.out.println("No April tag!");
            drivetrain.setControl(
                drive.withVelocityX(0) // Stop moving
                .withVelocityY(0)
                .withRotationalRate(0)
            );
        }
    }  
    public Command APT(){
        Command ret = run(()->{
            followAprilTag();
        });
        ret.addRequirements(drivetrain);
        return ret;
    }              
}