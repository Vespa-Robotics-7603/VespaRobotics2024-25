// package frc.robot.commands;

// import frc.robot.subsystems.Vision;
// import frc.robot.generated.TunerConstants;
// import frc.robot.subsystems.CommandSwerveDrivetrain;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.RunCommand;

// import static edu.wpi.first.units.Units.MetersPerSecond;
// import static edu.wpi.first.units.Units.RadiansPerSecond;
// import static edu.wpi.first.units.Units.RotationsPerSecond;

// import com.ctre.phoenix6.swerve.SwerveRequest;

// import edu.wpi.first.math.geometry.Translation2d;

// public class FollowAprilTagCommand extends RunCommand {
//     private final Vision visionSubsystem;
//     private final CommandSwerveDrivetrain drivetrain;
//     private static final double TARGET_DISTANCE_METERS = 1.0; // Stop at 1 meter from the tag

//     private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
//     private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

//     public FollowAprilTagCommand(Vision visionSubsystem, CommandSwerveDrivetrain drivetrain) {
//         this.visionSubsystem = visionSubsystem;
//         this.drivetrain = drivetrain;
//         addRequirements(visionSubsystem, drivetrain);
//     }

//     @Override
//     public void execute() {
//         var target = visionSubsystem.getBestTarget();
        
//         if (target != null) {
//             // Get distance and angle to the AprilTag
//             double targetYaw = visionSubsystem.getTargetYaw(); // Degrees
//             double targetDistance = visionSubsystem.getTargetDistance(); // Meters

//             // Compute movement speeds
//             double forwardSpeed = Math.max(0.2, Math.min(1.0, (targetDistance - TARGET_DISTANCE_METERS) * MaxSpeed)); // Speed scales based on distance
//             double rotationSpeed = -targetYaw * MMaxAngularRate; // Scale yaw to rotation (negative to correct direction)

//             // Create a movement request
//             drivetrain.applyRequest(() -> 
//                 new SwerveRequest.FieldCentric()
//                     .withVelocityX(forwardSpeed) // Move forward
//                     .withVelocityY(0) // No lateral movement
//                     .withRotationalRate(rotationSpeed) // Rotate to align with the tag
//             );
//         }
//     }

//     @Override
//     public boolean isFinished() {
//         double targetDistance = visionSubsystem.getTargetDistance();
//         return targetDistance > 0 && targetDistance <= TARGET_DISTANCE_METERS;
//     }

//     @Override
//     public void end(boolean interrupted) {
//         drivetrain.applyRequest(() -> new SwerveRequest.SwerveDriveBrake()); // Stop the robot
//     }
// }