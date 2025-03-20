package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;

//this gives me an error so I can't look at it's methods, but it builds
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.VictorSPXConfiguration;

import com.ctre.phoenix.motorcontrol.ControlMode;


public class CoralIntake implements Subsystem {
    
    public static CoralIntake singleInst;
    public static CoralIntake getInst(){
        if (singleInst == null) singleInst = new CoralIntake();
        return singleInst;
    }
    
    VictorSPX intakeMotor = new VictorSPX(9);
    double turnInRot = 0.3;
    double turnOutRot = 1;
    double refVal = 0;
    ControlMode controlM = ControlMode.Position;
    
    double turnInSpeed = 0.3;
    double turnOutSpeed = -0.3;
    
    public CoralIntake(){
        //TODO setup victor spx
        //setting pids (slot id, value)
        // intakeMotor.config_kP(0,0.1);
        // intakeMotor.config_kI(0, 0.1);
        // intakeMotor.config_kD(0,1);
        //feed forwards I assume
        // intakeMotor.config_kF(0, 0);
        //velocity in sensor units per 100 ms???? ew
        // intakeMotor.configMotionCruiseVelocity(2);
        //for all configs
        // VictorSPXConfiguration conf = new VictorSPXConfiguration();
        // conf.slot0.kP = 0; //I don't think this should be used
        // conf.primaryPID. // nor should this
        // driveLeft1.configAllSettings(conf);
    }
    
    public void CoralIn(){
        intakeMotor.set(ControlMode.PercentOutput, turnInSpeed);
    }
    
    public void CoralOut(){
        intakeMotor.set(ControlMode.PercentOutput, turnOutSpeed);
    }
    
    public void CoralStop() {
        intakeMotor.set(ControlMode.PercentOutput, 0);
    }

    @Override
    public void periodic(){
        // intakeMotor.set(controlM, refVal);
    }
    
    public Command CoralInCommand(){
        return Commands.sequence(runOnce(this::CoralIn), new WaitCommand(0.5), runOnce(this::CoralStop));
    }
    
    public Command CoralOutCommand(){
        return Commands.sequence(runOnce(this::CoralOut), new WaitCommand(0.5), runOnce(this::CoralStop));
    }
}
