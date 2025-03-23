package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.SwerveUtils.RevMotor;
import frc.robot.SwerveUtils.RevMotor.RevMotorSetPosition;


public class Elevator implements Subsystem {
    
    RevMotorSetPosition upDownMotor;
    
    
    //TODO get actual max rotation when build is done
    // double maxRot = 100;
    //TODO get percent height of levels wanted
    double intakePos = 100;
    double l0 = 0;
    double l1 = 85;
    double l2 = 180;
    double l3 = 320;
    double[] levels = {l0, l1, l2, l3};
    int currentLevel = 0;
    //arm positions, one for intake, one for output
    // double[] armPositions = {0.2, 0.30};
    double currentheight = 40;

    public Elevator(){
        upDownMotor = (RevMotorSetPosition) new RevMotorSetPosition(
            new SparkMax(2, MotorType.kBrushless),
             true,
             levels
        ).setMaxRot(340)//TODO get actual max rotation
        .setMinRot(-10);
        
        SparkMaxConfig config = new SparkMaxConfig();
        config
            .inverted(true)
            .idleMode(IdleMode.kBrake);
        config.encoder
            .positionConversionFactor(1)//keeping in rotations
            .velocityConversionFactor(1.0/60.0);//Keep in rotaion per minute (ew) by default
        config.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(1.2, 0.0, 0.1);
            
        upDownMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        // SparkMaxConfig configArm = new SparkMaxConfig();
        // //TODO, find device id
        // armMotor = (RevMotorSetPosition) new RevMotorSetPosition(
        //     new SparkMax(1, MotorType.kBrushless),
        //      true,
        //      armPositions
        // ).setMaxRot(100)//TODO get actual max rotation
        // .setMinRot(1);
        
        // config
        //     .inverted(false)
        //     .idleMode(IdleMode.kBrake);
        // config.encoder
        //     .positionConversionFactor(1)//keeping in rotations
        //     .velocityConversionFactor(1.0/60.0);// convert to rotations per second
        // config.closedLoop
        //     .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        //     //TODO fix these
        //     .pid(1.0, 0.0, 0.0);
            
        // armMotor.configure(configArm, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        // SparkMaxConfig configIntake = new SparkMaxConfig();
        // //TODO, find device id
        // coralIntakeMotor = new RevMotor(
        //     new SparkMax(2, MotorType.kBrushless),
        //      true
        // ).setMaxRot(Double.POSITIVE_INFINITY)//TODO get actual max rotation
        // .setMinRot(Double.NEGATIVE_INFINITY);
        
        // config
        //     .inverted(false)
        //     .idleMode(IdleMode.kBrake);
        // config.encoder
        //     .positionConversionFactor(1)//keeping in rotations
        //     .velocityConversionFactor(1.0/60.0);// convert to rotations per second
        // config.closedLoop
        //     .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        //     //TODO fix these
        //     .pid(1.0, 0.0, 0.0);
            
        // coralIntakeMotor.configure(configIntake, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
    
    public void incrementHeight(double increment) {
        // TODO: bounds check
        currentheight += increment;
        System.out.println("New target elevator position: " + currentheight);
        upDownMotor.goToRotation(currentheight);
    }    
    
    public void goToHeightPercent(double percentHeight){
        upDownMotor.goToRotationPercent(percentHeight);
    }
    
    // level must range between 1 and 3 inclusive
    public void goToLevel(int level){
        upDownMotor.goToRotation(levels[level]);
    }
    
    public void moveElevator(double speed){
        upDownMotor.Motor.set(speed);
    }
    
    public Command moveElevatorCommand(double speed){
        return run(()->{moveElevator(speed);});
    }
    
    
    public void moveElavatorWithSpeed(double speed){
        upDownMotor.setSpeed(speed);
    }
    
    @Override
    public void periodic(){
        upDownMotor.resetReference();
        // armMotor.resetReference();
        // coralIntakeMotor.resetReference();
        //System.out.println("Running Periodic");
        // System.out.println("Elevator position: " + upDownMotor.Motor.getEncoder().getPosition());
    }
    //Using run once here because the motor will continue to go to position/speed
    //it doesn't need to be called periodically, only when a change in motion is wanted
    public Command goToHeightPercentCommand(double percentHeight){
        return runOnce( ()-> {goToHeightPercent(percentHeight);});
    }
    
    public Command goToLevelCommand(int level){
        return runOnce( ()->{goToLevel(level);});
    }
    
    public Command moveElavatorWithSpeedCommand(double speed){
        return runOnce(()->{moveElavatorWithSpeed(speed);});
    }
    
    @Deprecated
    public Command goToLevel2AndOutputTest(){
        //test command chaining for outputting coral on level 2
        Command f = new Command() {
            @Override
            public void execute(){
                goToLevel(2);
            }
            
            @Override
            public boolean isFinished(){
                double elevatorRot = upDownMotor.Motor.getEncoder().getPosition();
                double wantedRot = upDownMotor.getRotationsFromPercent(l2);
                double withinRange = 0.2;
                if( 
                    elevatorRot > wantedRot-withinRange
                    && elevatorRot < wantedRot+withinRange
                ){
                    // within range, should end
                    return false;
                }
                return true;
            }
        };
        f.addRequirements(this);
        
        Command d = new Command() {
            @Override
            public void execute(){
                // armOutput();
            }
            
            @Override
            public boolean isFinished(){
                // double armRot = armMotor.Motor.getEncoder().getPosition();
                // double wantedRot = armMotor.getRotationsFromPercent(l2);
                // double withinRange = 0.2;
                // if( 
                //     armRot > wantedRot-withinRange
                //     && armRot < wantedRot+withinRange
                // ){
                //     // within range, should end
                //     return false;
                // }
                return true;
            }
            
            @Override
            public void end(boolean interrupted){
                if(!interrupted){
                    // CoralOut();
                }
            }
        };
        
        
        d.addRequirements(this);
        
        return f.andThen(d);
    }

    public Command oneLevelUp(){
        return runOnce(()->{
            if(currentLevel < 3){
                currentLevel++;
            }
            
            goToLevel(currentLevel);
            
        });
    }
    public Command oneLevelDown(){
        return runOnce(()->{
            if(currentLevel > 0){
                currentLevel--;
            }
            goToLevel(currentLevel);
        });
        
    }

    public Command positionIncrementCommand(double height) {
        return runOnce(() -> {
            incrementHeight(height);
        });
    }

    public Command toStart(){
        return runOnce(() ->{
            currentLevel = 0;
            upDownMotor.goToStart();
        });
    }
    
    
    public Command RunCommandToDoThing(){
        return run(()->{
            //spin I guess
        });
    }
}
