package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.SwerveUtils.RevMotor;

import frc.robot.constants.AlgaeConstants;

public class Algae implements Subsystem { // i love big green balls
    RevMotor algaeMotor;
    public Algae() {
        algaeMotor = new RevMotor(4, MotorType.kBrushless);
    }
    
    @Override
    public void periodic(){
        // algaeMotor.resetReference();
    }
    
    public Command AlgaeIn(){
        return run(()->{
            algaeMotor.Motor.set(AlgaeConstants.ALGAE_IN);
        });
    }
    
    public Command AlgaeOut(){
        return run(() -> {
            algaeMotor.Motor.set(AlgaeConstants.ALGAE_OUT);
        });
    }
    
    public void stop(){
        algaeMotor.Motor.set(0);
    }
    public Command AlgeaStopCommand(){
        return run(this::stop);
    }
}
