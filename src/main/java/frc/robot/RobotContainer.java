// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.SwerveUtils.AprilTagFollower;
import frc.robot.SwerveUtils.AprilTagFollower.FollowTagData;
import frc.robot.constants.AprilTagData;
import frc.robot.SwerveUtils.TrajectoryTarget2d;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Algae;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoralIntake;
import frc.robot.subsystems.CoralPivot;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.TrajectoryFollower;
import frc.robot.subsystems.Vision;

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
    private final SwerveRequest.FieldCentric driveLowerSpeed = new SwerveRequest.FieldCentric()
            .withDeadband((MaxSpeed/2) * 0.1).withRotationalDeadband((MaxAngularRate/2) * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    // Subsystems
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final Elevator elevator = Elevator.getInst();
    public final CoralPivot arm = CoralPivot.getInst();
    public final CoralIntake coral = CoralIntake.getInst();
    public final Algae algae = Algae.getInst();
    public final Vision photonVision = new Vision(drivetrain);
    
    public final TrajectoryFollower follower = new TrajectoryFollower(drivetrain);
    public final AprilTagFollower APTFollower = new AprilTagFollower(photonVision, follower);
    
    /** 
     * The current mode of the controler
     * (this allows for more bindings without another contorller) 
     */
    private short CurrentMode = 0;

    // private final Vision visionSubsystem = new Vision(drivetrain);

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        CommandScheduler.getInstance().registerSubsystem(elevator, arm, coral, algae, photonVision);

        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() -> drive
                .withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );
        
        CommandScheduler.getInstance().registerSubsystem(elevator, arm, coral, algae);


        //DriverStation.startDataLog(DataLogManager.getLog());
        algae.setDefaultCommand(algae.AlgaeStopCommand());
        coral.setDefaultCommand(coral.holdCom());
        
        Command upOneLevel = Commands.parallel(
            elevator.oneLevelUp(), arm.toOutput()
        );
        
        Command downOneLevel = Commands.parallel(
            elevator.oneLevelDown(), arm.toOutput()
        );
        
        Command toInputPosition = Commands.parallel(
            elevator.toInputLevel(), arm.toIntake()
        );
        
        //OLD ELEVATOR COMMANDS
        // joystick.povUp().onTrue(elevator.oneLevelUp());
        // joystick.povDown().onTrue(elevator.oneLevelDown());
        
        
        //OLD ARM COMMANDS
        // joystick.povLeft().onTrue(arm.toIntake());
        // joystick.povRight().onTrue(arm.toOutput());
        
        //POV/D-PAD COMMANDS
        joystick.povUp().onTrue(upOneLevel);
        joystick.povDown().onTrue(downOneLevel);
        
        joystick.povLeft().onTrue(arm.toIntake());
        joystick.povRight().onTrue(toInputPosition);
        
        //TRIGGER COMMANDS
        joystick.leftTrigger(0.1).whileTrue(coral.CoralIn(joystick::getLeftTriggerAxis));
        joystick.rightTrigger(0.1).whileTrue(coral.CoralOut(joystick::getRightTriggerAxis));
        
        //BUMPER COMMANDS
        joystick.rightBumper().whileTrue(algae.AlgaeIn());
        joystick.leftBumper().whileTrue(algae.AlgaeOut());

        // reset the field-centric heading on menu press
        joystick.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        joystick.start().onTrue(Commands.print("drive train reset! :) so gracious! so professional!"));

        // TODO: don't hardcode alignment values
        Command intakecommand = APTFollower.alignWithTag(1, 
            // new FollowTagData(11, 0.5, 0.16, Math.PI, joystick.y())
            FollowTagData.constructTagDatas(0.5, -0.3, Math.PI + Math.PI / 4, joystick.y(), AprilTagData.INTAKE_TAGS)
        );

        Command leftcoralcommand = APTFollower.alignWithTag(1, 
            // new FollowTagData(11, 0.5, 0.16, Math.PI, joystick.y())
            FollowTagData.constructTagDatas(0.5, 0.16, Math.PI, joystick.y(), AprilTagData.REEF_TAGS)
        );

        Command rightcoralcommand = APTFollower.alignWithTag(1, 
            // new FollowTagData(11, 0.5, 0.16, Math.PI, joystick.y())
            FollowTagData.constructTagDatas(0.5, -0.16, Math.PI, joystick.y(), AprilTagData.REEF_TAGS)
        );
        
        joystick.a().onTrue(intakecommand);
        joystick.b().onTrue(leftcoralcommand);
        joystick.x().onTrue(rightcoralcommand);
        // drivetrain.resetPose(null);
        
        // joystick.b().onTrue(
        //     follower.wrapGeneratedCommand(
        //         new TrajectoryTarget2d(
        //             1, 0, 0
        //         )
        //     )
        // );
        
        // joystick.b().whileTrue(
        //     drivetrain.applyRequest(() -> driveLowerSpeed
        //         .withVelocityX(-joystick.getLeftY() * (MaxSpeed/2)) // Drive forward with negative Y (forward)
        //         .withVelocityY(-joystick.getLeftX() * (MaxSpeed/2)) // Drive left with negative X (left)
        //         .withRotationalRate(-joystick.getRightX() * (MaxAngularRate/2)) // Drive counterclockwise with negative X (left)
        //     )
        // );

        //GENERATED COMMANDS (can be replaced)
        // joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        // joystick.b().whileTrue(drivetrain.applyRequest(() ->
        //     point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        // ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        NamedCommands.registerCommand("Elevator Up", elevator.toAutoOutput());
        NamedCommands.registerCommand("Coral Out", coral.CoralOutCom());
        NamedCommands.registerCommand("Roll Off", algae.RollOffCommand());
        NamedCommands.registerCommand("Output Position", arm.toOutput());

        //for Cycle 2-3 of auto
        NamedCommands.registerCommand("Input Level", elevator.toInputLevel());
        NamedCommands.registerCommand("Coral In", coral.CoralInCom());
        NamedCommands.registerCommand("Input Position", arm.toIntake());
        return new PathPlannerAuto("Right Auto");

        //return Commands.print("No autonomous command configured");
        // TrajectoryTarget2d targetinfo = new TrajectoryTarget2d(1, 1, Math.PI / 2);
        // return new TrajectoryFollower(drivetrain).moveToTarget(3, 3, targetinfo);
        //return new PathPlannerAuto("1cM");
        //return new PathPlannerAuto("Test Auto");
    }
    
    /**
     * To be used in with tiggers
      */
    public BooleanSupplier getIsModeEqualTo(short modeValue){
        return ()->CurrentMode == modeValue;
    }
    
    public Command changeCurrentModeTo(short newValue){
        return Commands.runOnce(()->{
            CurrentMode=newValue;
            System.out.println("Mode = " + CurrentMode);
        });
    }
}