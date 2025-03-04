// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;



import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;


/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    public Robot() {
        //TODO configure pids per motor
        // SparkMaxConfig config = new SparkMaxConfig();
        // config
        //     .inverted(false)
        //     .idleMode(IdleMode.kBrake);
        // config.encoder
        //     .positionConversionFactor(1)//keeping in rotations
        //     .velocityConversionFactor(1);//Keep in rotaion per minute (ew) by default
        // config.closedLoop
        //     .feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        //     // .pid(0.0, 0.0, 0.0);
        // motor0.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
    }
    
    //TODO label motors and test movement
    //TODO find desired rotation numbers
    Joystick joystick = new Joystick(0);
    SparkMax motor0 = new SparkMax(0, MotorType.kBrushless);
    SparkMax motor1 = new SparkMax(1, MotorType.kBrushless);
    SparkMax motor2 = new SparkMax(1, MotorType.kBrushless);
    SparkMax motor3 = new SparkMax(3, MotorType.kBrushless);
    SparkMax motor4 = new SparkMax(4, MotorType.kBrushless);
    
    SparkMax[] motors = {motor0, motor1, motor2, motor3, motor4};
    double[] motorSpeeds = {0.2, 0.2, 0.2, 0.2, 0.2};

    @Override
    public void robotPeriodic() {}

    @Override
    public void autonomousInit() {}

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {}
    
    boolean whichPos = true;
    private void setRef(SparkMax motorToTest){
        double position1 = 0;
        double position2 = 0;
        if (joystick.getTriggerPressed()) {
            whichPos = !whichPos;
        }
        System.out.println("Setting ref!");
        motorToTest.getClosedLoopController()
            .setReference(
                whichPos? position1:position2
            , ControlType.kPosition
            );
    }

    @Override
    public void teleopPeriodic() {
        for (int i = 0; i < motors.length; i++) {
            motors[i].set(0);
            joystick.setRumble(RumbleType.kBothRumble, 0);
            System.out.println(
                "Motor id: " + i +
                "\nRotations: " + motors[i].getEncoder().getPosition()
            );
            
            if (joystick.getRawButton(2*i+1)) {
                joystick.setRumble(RumbleType.kRightRumble, 0.1);
                motors[i].set(motorSpeeds[i]);
            }
            else if(joystick.getRawButton(2*i+2)){
                joystick.setRumble(RumbleType.kLeftRumble, 0.1);
                motors[i].set(-motorSpeeds[i]);
            }
        }
        // setRef(motor0);
        
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void testInit() {}

    @Override
    public void testPeriodic() {}

    @Override
    public void simulationInit() {}

    @Override
    public void simulationPeriodic() {}
}
