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

import static frc.robot.constants.CoralConstants.*;

public class CoralPivot implements Subsystem{
    
    private static CoralPivot singleInst;
    /**
     * Returns the instance of this subsystem.
     * <p>This is to stop multiple instances of motors 
     * and allow better access to the subsystem</p>
     * @return the instance.
      */
    public static CoralPivot getInst(){
        return (singleInst == null)? new CoralPivot(): singleInst;
    }
    
    
    RevMotorSetPosition armMotor;
    double currentPos = 0;
    
    CoralPivot(){
        SparkMaxConfig configArm = new SparkMaxConfig();
        
        armMotor = (RevMotorSetPosition) new RevMotorSetPosition(
            new SparkMax(3, MotorType.kBrushless),
            true,
            IN_POS, OUT_POS)
        .setMaxRot(MAX_ROT)
        .setMinRot(MIN_ROT);
        
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
    
    public void armOutput(){
        armMotor.goToSetPosition(1);
    }

    public void incrementHeight(double height) {
        currentPos += height;
        System.out.println("New target coral arm position: " + currentPos);
        armMotor.goToRotation(currentPos);
    }
    
    public void armWithSpeed(double speed){
        armMotor.setSpeed(speed);
    }
    
    @Override
    public void periodic(){
        armMotor.resetReference();
        // System.out.println("Arm motor position: " + armMotor.Motor.getEncoder().getPosition());
    }

    public Command positionIncrementCommand(double increment) {
        return runOnce(() -> {
            incrementHeight(increment);
        });
    }

    public Command toOutput(){
        return runOnce(() -> {
            armOutput();
        });
    }

    public Command toIntake(){
        return runOnce(() -> {
            armIntake();
        });
    }
    
    public Command setSpeed(double speed){
        return runOnce(() -> {
            this.setSpeed(speed);
        });
    }
    
    public void moveArm(double speed){
        armMotor.Motor.set(speed);
    }
    
    public Command moveArmCommand(double speed){
        return run(()->{moveArm(speed);});
    }

    
}
