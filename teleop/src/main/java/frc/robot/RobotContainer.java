// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
//import frc.robot.commands.FollowAprilTagCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Algae;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoralBucket;
import frc.robot.subsystems.CoralPivot;
import frc.robot.subsystems.Elevator;

// import frc.robot.subsystems.Vision;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    // Subsystems
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final Elevator elevator = new Elevator();
    public final CoralPivot arm = new CoralPivot();
    public final CoralBucket coral = new CoralBucket();
    public final Algae algae = new Algae();

    // private final Vision visionSubsystem = new Vision(drivetrain);

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );
        
        algae.setDefaultCommand(algae.AlgeaStopCommand());
        coral.setDefaultCommand(coral.holdCom());

        CommandScheduler.getInstance().registerSubsystem(elevator, arm, coral, algae);
        
        joystick.povUp().onTrue(elevator.oneLevelUp());
        joystick.povDown().onTrue(elevator.oneLevelDown());

        joystick.povLeft().whileTrue(arm.toIntake());
        joystick.povRight().whileTrue(arm.toOutput());
        
        joystick.rightTrigger().whileTrue(coral.CoralInCom()); //todo: make this dynamic and not a button
        joystick.leftTrigger().whileTrue(coral.CoralOutCom());

        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));


        joystick.rightBumper().whileTrue(algae.AlgaeIn());
        joystick.leftBumper().whileTrue(algae.AlgaeOut());

        // reset the field-centric heading on menu press
        joystick.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        joystick.start().onTrue(Commands.print("drive train reset! :) so gracious! so professional!"));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        //return Commands.print("No autonomous command configured");
        // return visionSubsystem.followAprilTag();
        return null;
    }
}