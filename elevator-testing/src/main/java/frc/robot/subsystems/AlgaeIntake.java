package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.SwerveUtils.RevMotor;

public class AlgaeIntake implements Subsystem {
    RevMotor algeaMotor;
    
    public AlgaeIntake(){
        
        algeaMotor = new RevMotor(4, MotorType.kBrushless);
    }
    
    public void AlgaeIn(){
        algeaMotor.Motor.set(0.9);
    }
    public void AlgaeOut(){
        algeaMotor.Motor.set(-0.9);
    }
    
    @Override
    public void periodic(){
        //algeaMotor.resetReference();
    }
    
    public Command AlgaeInCommand(){
        return run(()->{
            AlgaeIn();
        });
    }
    public void stop(){
        algeaMotor.Motor.set(0);
    }
    
    public Command AlgaeOutCommand(){
        return run(() -> {
            AlgaeOut();
        });
    }
    public Command AlgeaStopCommand(){
        return run(this::stop);
    }
}
