package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.SwerveUtils.RevMotor.RevMotorSetPosition;

public class CoralPivot implements Subsystem{
    
    public static CoralPivot singleInst;
    public static CoralPivot getInst(){
        if (singleInst == null) singleInst = new CoralPivot();
        return singleInst;
    }
    
    public RevMotorSetPosition armMotor;
    //arm positions, one for intake, one for output
    double outPos = 7.83;
    double inPos = 13.12;
    // double[] armPositions = {outPos, inPos};
    
    public CoralPivot(){
        SparkMaxConfig configArm = new SparkMaxConfig();
        
        armMotor = (RevMotorSetPosition) new RevMotorSetPosition(
            new SparkMax(3, MotorType.kBrushless),
             true,
             inPos, outPos
        ).setMaxRot(13)//TODO get actual max rotation
        .setMinRot(-1);
        
        configArm
            .inverted(true)
            .idleMode(IdleMode.kBrake);
        configArm.encoder
            .positionConversionFactor(1)//keeping in rotations
            .velocityConversionFactor(1.0/60.0);// convert to rotations per second
        configArm.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(0.05, 0.0, 0.0)
            .maxMotion.maxVelocity(2);

            
            
        armMotor.configure(configArm, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
    }
    
    public void armIntake(){
        armMotor.goToSetPosition(0);
    }
    
    public void armOutput(int level){
        armMotor.goToSetPosition(1);
    }
    
    @Override
    public void periodic(){
        armMotor.resetReference();
        // System.out.println(armMotor.Motor.getEncoder().getPosition());
    }
    public Command toOutput(){
        return runOnce(() -> {
            armOutput(0);
        });
    }
    public Command toIntake(){
        return runOnce(() -> {
            armIntake();
        });
    }

    
}
