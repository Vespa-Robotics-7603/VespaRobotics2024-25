package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.SwerveUtils.RevMotor;

import static frc.robot.constants.AlgaeConstants.*;

public class Algae implements Subsystem { // i love big green balls
    
    private static Algae singleInst;
    /**
     * Returns the instance of this subsystem.
     * <p>This is to stop multiple instances of motors 
     * and allow better access to the subsystem</p>
     * @return the instance.
      */
    public static Algae getInst(){
        return (singleInst == null)? new Algae(): singleInst;
    }
    
    RevMotor algaeMotor;
    
    Algae() {
        algaeMotor = new RevMotor(4, MotorType.kBrushless);
    }
    
    @Override
    public void periodic(){
        // algaeMotor.resetReference();
    }
    
    public Command AlgaeIn(){
        return run(()->{
            algaeMotor.Motor.set(ALGAE_IN);
        });
    }
    
    public Command AlgaeOut(){
        return run(() -> {
            algaeMotor.Motor.set(ALGAE_OUT);
        });
    }
    
    public void stop(){
        algaeMotor.Motor.set(0);
    }
    public Command AlgaeStopCommand(){
        return run(this::stop);
    }
    
    public Command RollOffCommand(){
        
        return Commands.race(
            AlgaeOut(),
            new WaitCommand(1)
        ).andThen(runOnce(this::stop));
    }
}
