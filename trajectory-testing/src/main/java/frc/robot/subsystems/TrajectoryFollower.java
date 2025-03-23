package frc.robot.subsystems;

import java.util.List;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.constraint.SwerveDriveKinematicsConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

import static java.lang.Math.PI;

public class TrajectoryFollower {
    CommandSwerveDrivetrain drivetrain;
    SwerveDriveKinematicsConstraint constraint;
    HolonomicDriveController controller;
    
    public TrajectoryFollower(CommandSwerveDrivetrain drivetrain){
        this.drivetrain = drivetrain;
        constraint = 
        new SwerveDriveKinematicsConstraint(drivetrain.getKinematics(), maxSpeed);
        // Create a voltage constraint to ensure we don't accelerate too fast
        controller = new HolonomicDriveController(
            new PIDController(0.11, 0, 0), 
            new PIDController(0.18, 0, 0),
            new ProfiledPIDController(11, 0, 0,
                new TrapezoidProfile.Constraints(maxSpeed, 3.14)
            )
        );
    }
    
    double maxSpeed = 3;
    
    public Command FollowCommand(){

        // Create config for trajectory
        TrajectoryConfig config =
            new TrajectoryConfig(
                maxSpeed,
                3.14
            )
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(drivetrain.getKinematics())
            // Apply the voltage constraint
            .addConstraint(constraint);
                

        // An example trajectory to follow. All units in meters.
        Trajectory exampleTrajectory =
            TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0, 0, new Rotation2d(0)),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(/* new Translation2d(1, 1), new Translation2d(2, -1) */),
                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(0, 1, new Rotation2d(PI/2)),
                // Pass config
                config);
        // Trajectory f= 

        SwerveControllerCommand command = new SwerveControllerCommand(
            exampleTrajectory, 
            drivetrain::getPose, 
            drivetrain.getKinematics(), 
            controller, 
            this::moveWithState, 
            drivetrain
        );

        // Reset odometry to the initial pose of the trajectory, run path following
        // command, then stop at the end.
        return Commands.runOnce(() ->{
            //reset odometry?
            // drivetrain.tareEverything();
            System.out.println("Reset odo here");
        } )
            .andThen(command)
            .andThen(Commands.runOnce(() -> {
                //brake
                System.out.println("BRAKING");
                drivetrain.setControl(new SwerveRequest.SwerveDriveBrake());
            })
        );
    }
    
    private void moveWithState(SwerveModuleState... states){
        //TODO
        System.out.println("Applying speeds...");
        drivetrain.setControl(
            new SwerveRequest.ApplyRobotSpeeds().withSpeeds(
                drivetrain.getKinematics().toChassisSpeeds(states)
            )
        );
    }
    
    public Command followTraj(Trajectory trajectoryToFollow){
        
        return new SwerveControllerCommand(
            trajectoryToFollow, 
            drivetrain::getPose, 
            drivetrain.getKinematics(), 
            controller, 
            this::moveWithState, 
            drivetrain
        );
    }
}
