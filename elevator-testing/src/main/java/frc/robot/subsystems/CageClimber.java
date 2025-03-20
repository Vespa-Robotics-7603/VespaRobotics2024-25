package frc.robot.subsystems;


import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.SwerveUtils.RevMotor;

//TODO: MAKE SURE THIS STOPS!!!!!
public class CageClimber implements Subsystem{
    boolean climbed = false;
    public static CageClimber singleInst;
    public static CageClimber getInst(){
        if (singleInst == null) singleInst = new CageClimber();
        return singleInst;
    }
    
    RevMotor footMotor;
    
    public CageClimber(){
        //MAX CLIMBER MOTOR 117.66

        SparkMaxConfig config = new SparkMaxConfig();
        
        footMotor = new RevMotor(
            new SparkMax(5, MotorType.kBrushless), true
        ).setMaxRot(115)
        .setMinRot(-1);
        
        config
            .inverted(false)
            .idleMode(IdleMode.kBrake);
        config.encoder
            .positionConversionFactor(1)//keeping in rotations
            .velocityConversionFactor(1.0/60.0);// convert to rotations per second
        config.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            //TODO fix these
            .pid(1.0, 0.0, 0.0);
            
        footMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
    
    public void down(){
        footMotor.goToStart();
    }
    public void up(){
        footMotor.goToRotationPercent(0.99);
    }
    
    @Override
    public void periodic(){
        footMotor.resetReference();
    }
    
    public Command climbCommand(){
        return runOnce(this::down);
    }
    
    public Command upCommand(){
        return runOnce(this::up);
    }
    public Command climber(){
        return runOnce(() -> {
            if(!climbed){
                down();
                climbed = true;
            }else {
                up();
                climbed = false;

            }
        });
    }
    
}
