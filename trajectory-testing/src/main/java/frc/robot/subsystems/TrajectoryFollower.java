package frc.robot.subsystems;


import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.constraint.SwerveDriveKinematicsConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.SwerveUtils.TrajectoryTarget2d;

// TODO: Document and test everything, refactor if necessary.
// TODO: Javadoc everything once code implementation is finalized
public class TrajectoryFollower {
    private final double MAX_SPEED = 10;
    private final double MAX_ROTATIONAL_SPEED = 4;
    private final double MAX_ROTATIONAL_ACCELERATION = 3.14;
    CommandSwerveDrivetrain drivetrain;
    SwerveDriveKinematicsConstraint constraint;
    HolonomicDriveController controller;

    // TODO: Determine our robot's ideal max speed.
    public TrajectoryFollower(CommandSwerveDrivetrain drivetrain){
        this.drivetrain = drivetrain;
        constraint = new SwerveDriveKinematicsConstraint(
            drivetrain.getKinematics(),
            MAX_SPEED
        );
        // Create a voltage constraint to ensure we don't accelerate too fast
        controller = new HolonomicDriveController(
            new PIDController(0.11, 0, 0), 
            new PIDController(0.18, 0, 0),
            new ProfiledPIDController(11, 0, 0,
                new TrapezoidProfile.Constraints(MAX_ROTATIONAL_SPEED, MAX_ROTATIONAL_ACCELERATION)
            )
        );
    }

    // TODO: Reformat this code so that it's more readable.
    public TrajectoryConfig generateTrajectoryConfig(double speed, double acceleration) {
        // Create config for trajectory
        return new TrajectoryConfig(
                speed,
                acceleration
            )
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(drivetrain.getKinematics())
            // Apply the voltage constraint
            .addConstraint(constraint);
    }

    public Trajectory generateTrajectory(double speed, double acceleration, TrajectoryTarget2d target) {
        TrajectoryConfig config = generateTrajectoryConfig(speed, acceleration);
        return TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            target.getWaypoints(),
            new Pose2d(target.getTranslation(), target.getRotation()),
            config
        );
    }
    
    public SwerveControllerCommand generateMovementCommand(Trajectory trajectory) {
        return new SwerveControllerCommand(
            trajectory,
            drivetrain::getPose, 
            drivetrain.getKinematics(), 
            controller,
            this::moveWithState, 
            drivetrain
        );
    }

    public Command moveToTarget(double speed, double acceleration, TrajectoryTarget2d target){
        Trajectory trajectory = generateTrajectory(speed, acceleration, target);
        SwerveControllerCommand command = generateMovementCommand(trajectory);
        return command;
    }
    
    private void moveWithState(SwerveModuleState... states){
        //TODO
        // Editor's note: next time, please be more specific with your TODOs...
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
