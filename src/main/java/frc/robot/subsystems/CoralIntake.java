package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;

//this gives me an error so I can't look at it's methods, but it builds
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
// import com.ctre.phoenix.motorcontrol.can.VictorSPXConfiguration;

import com.ctre.phoenix.motorcontrol.ControlMode;

import static frc.robot.constants.CoralConstants.*;

import java.util.function.Supplier;


public class CoralIntake implements Subsystem {
    
    private static CoralIntake singleInst;
    /**
     * Returns the instance of this subsystem.
     * <p>This is to stop multiple instances of motors 
     * and allow better access to the subsystem</p>
     * @return the instance.
      */
    public static CoralIntake getInst(){
        return (singleInst == null)? new CoralIntake(): singleInst;
    }
    
    VictorSPX intakeMotor = new VictorSPX(9);
    
    CoralIntake(){}
    
    public void CoralIn(){
        intakeMotor.set(ControlMode.PercentOutput, TURN_IN_ROT);
    }
    public void CoralOut(){
        intakeMotor.set(ControlMode.PercentOutput, TURN_OUT_ROT);
    }
    public Command CoralIn(Supplier<Double> triggerAxis){
        return run(() -> {
            intakeMotor.set(ControlMode.PercentOutput, triggerAxis.get()*TURN_IN_ROT);
        });
    }
    public Command CoralOut(Supplier<Double> triggerAxis){
        return run(() -> {
            intakeMotor.set(ControlMode.PercentOutput, triggerAxis.get()*TURN_OUT_ROT);
        });
    }
    
    public void hold(){
        // System.out.println("HOLDING!!!!");
        intakeMotor.set(ControlMode.PercentOutput, CORAL_HOLD);
    }
    
    public Command holdCom(){
        return run(this::hold);
    }
    
    @Override
    public void periodic(){
        //intakeMotor.set(controlM, refVal);
        /* Ethan = breakfast by half alive 
         * Harley = BALLZ!
         * Seysha = supper's ready by genisis
         * -harley & ethan
         */
    }
    public void CoralStop(){
        intakeMotor.set(ControlMode.PercentOutput, 0);
    }

    public Command CoralInCom(){
        return Commands.sequence(runOnce(this::CoralIn), new WaitCommand(IN_WAIT_SEC), runOnce(this::CoralStop));
    }
    
    public Command CoralOutCom(){
        return Commands.sequence(runOnce(this::CoralOut), new WaitCommand(OUT_WAIT_SEC), runOnce(this::CoralStop));
    }
}
