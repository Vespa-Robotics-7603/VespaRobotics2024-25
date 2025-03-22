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
    
    
    RevMotorSetPosition armMotor;
    //arm positions, one for intake, one for output
    double outPos = 6.4;
    double inPos = 13;
    double currentposition = 0;
    // double[] armPositions = {outPos, inPos};
    
    public CoralPivot(){
        SparkMaxConfig configArm = new SparkMaxConfig();
        
        armMotor = (RevMotorSetPosition) new RevMotorSetPosition(
            new SparkMax(3, MotorType.kBrushless),
             true,
             outPos, inPos
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

    public void incrementHeight(double height) {
        // TODO: bounds check
        currentposition += height;
        System.out.println("New target coral arm position: " + currentposition);
        armMotor.goToRotation(currentposition);
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
            armOutput(0);
        });
    }

    public Command toIntake(){
        return runOnce(() -> {
            armIntake();
        });
    }
    
    public Command setSpeed(double speed){
        return runOnce(()->{
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
