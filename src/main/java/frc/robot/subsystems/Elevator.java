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
import frc.robot.SwerveUtils.RevMotor.RevMotorSetPosition;

import static frc.robot.constants.ElevatorConstants.*;


public class Elevator implements Subsystem {
    
    private static Elevator singleInst;
    /**
     * Returns the instance of this subsystem.
     * <p>This is to stop multiple instances of motors 
     * and allow better access to the subsystem</p>
     * @return the instance.
      */
    public static Elevator getInst(){
        return (singleInst == null)? new Elevator(): singleInst;
    }
    
    RevMotorSetPosition upDownMotor;
    
    int currentLevel = 0;
    
    double currentheight = 40;

    Elevator(){
        upDownMotor = (RevMotorSetPosition) new RevMotorSetPosition(
            new SparkMax(2, MotorType.kBrushless),
             true,
             GROUND_LEVEL, L2, L3
        ).setMaxRot(MAX_ROT)
        .setMinRot(MIN_ROT);
        
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
    
    // level must range between 0 and 3 inclusive
    public void goToLevel(int level){
        currentLevel = level;
        upDownMotor.goToSetPosition(level);
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
    
    public Command toInputLevel(){
        //subject to change it new input level is found
        return goToLevelCommand(0);
    }

    public Command toAutoOutput(){
        return goToLevelCommand(1);
        
    }
}
