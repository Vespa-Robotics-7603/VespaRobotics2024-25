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
    
    public static Elevator singleInst;
    public static Elevator getInst(){
        if (singleInst == null) singleInst = new Elevator();
        return singleInst;
    }
    
    RevMotorSetPosition upDownMotor;

    double l0 = 5;
    double l1 = 110;
    double l2 = 210;
    double l3 = 333;
    double intake = 5;
    double[] levels = {l0,l1, l2, l3};
    int currentLevel = 0;
    
    public Elevator(){
        upDownMotor = (RevMotorSetPosition) new RevMotorSetPosition(
            new SparkMax(2, MotorType.kBrushless),
             true,
             levels
        ).setMaxRot(325)
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
            .pid(5, 0.0, 0);
            
        upDownMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
    
    
    public void goToHeightPercent(double percentHeight){
        upDownMotor.goToRotationPercent(percentHeight);
    }
    
    // level must range between 1 and 3 inclusive
    public void goToLevel(int level){
        upDownMotor.goToRotation(levels[level]);
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
            System.out.println("currentLevel UP to:"+currentLevel);
            goToLevel(currentLevel);
            
        });
    }
    
    public Command oneLevelDown(){
        return runOnce(()->{
            if(currentLevel > 0){
                currentLevel--;
                System.out.println("currentLevel Down to:"+currentLevel);

            }
            goToLevel(currentLevel);
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
    public Command toIntake(){
        return runOnce(() ->{
            upDownMotor.goToRotation(intake);
        });
    }
}
