package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.SwerveUtils.RevMotor;

public class AlgaeIntake implements Subsystem {
    
    public static AlgaeIntake singleInst;
    public static AlgaeIntake getInst(){
        if (singleInst == null) singleInst = new AlgaeIntake();
        return singleInst;
    }
    
    RevMotor algeaMotor;
    
    double algaeInSpeed = 0.3;
    double algaeOutSpeed =-0.3;
    
    public AlgaeIntake(){
        
        algeaMotor = new RevMotor(4, MotorType.kBrushless);
    }
    
    public void AlgaeIn(){
        algeaMotor.Motor.set(algaeInSpeed);
    }
    public void AlgaeOut(){
        algeaMotor.Motor.set(algaeOutSpeed);
    }
    public void Stop(){
        algeaMotor.Motor.set(0);;
    }
    
    @Override
    public void periodic(){
        //algeaMotor.resetReference();
    }
    
    public Command AlgaeInCommand(){
        return run(()->{
            System.out.println("Algea In");
            AlgaeIn();
        });
    }
    
    public Command AlgaeOutCommand(){
        return run(this::AlgaeOut);
    }
    
    public Command AlgaeStopCommand(){
        return run(this::Stop);
    }
}
