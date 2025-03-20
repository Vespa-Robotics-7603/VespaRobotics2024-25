package frc.robot.SwerveUtils;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

/**
 * Encasulaes methods and variable needed to control the revlib motors.
 * <p>
 * <b>NOTE:</b> motors are not automaticaly configured properly.
 * <br>
 * You will need to configure the motor yourself by:
 * </p>
 * <ul>
 *      <li>Passing the configured motor to the constuctor {@link #RevMotor(SparkMax, boolean)};</li>
 *      <li>Confuguring the motor later by acessing the {@link #Motor} field;
 *      <li>Or by using the {@link #configure(SparkMaxConfig, ResetMode, PersistMode)} method.
 * </ul>
 * <p>
 * <b>NOTE:</b> You need to call {@link #resetReference()} in any periodic method
 * (like teleop periodic or subystems perodic)
 * </p>
  */
public class RevMotor {
    /** The actual motor  */
    public SparkMax Motor;
    /** The Clossed loop controller, used to contorl speed */
    public SparkClosedLoopController CLController;
    /** Maximum rotations allowed for this motor  */
    protected double maxRot = 10;
    /** Minimum rotations allowed for this motor  */
    protected double minRot = -10;
    /** Offset used when minimum rotations is reached  */
    protected double minRotOffset = 0.1;
    /** Offset used when maximum rotations is reached  */
    protected double maxRotOffset = 0.1;
    
    /** Value used to control the motor.
     * Units and usdage depend on the current Control Type
     * @see #controlT
     */
    private double refVal = 0;
    /** The current Control Type. Determins how the motor should be controled 
     * @see #refVal
     */
    private ControlType controlT = ControlType.kPosition;
    
    /**
     * Constucts a motor.
     * @param deviceId The device ID
     * @param motorType The type of motor, brushed or burshless
      */
    public RevMotor(int deviceId, MotorType motorType){
        this(new SparkMax(deviceId, motorType), true);
        
    }
    
    /**
     * Uses the motor given and configures if requested.
     * @param motor The motor to use.
     * @param IsAlreadyConfigured Whether the motor is/will be configured
      */
    public RevMotor(SparkMax motor, boolean IsAlreadyConfigured){
        Motor = motor;
        CLController = Motor.getClosedLoopController();
        
        CLController.setReference(refVal, controlT);
        
        if(!IsAlreadyConfigured){
            SparkMaxConfig config = new SparkMaxConfig();
            config
                .inverted(false)
                .idleMode(IdleMode.kBrake);
            config.encoder
                .positionConversionFactor(1)//keeping in rotations
                .velocityConversionFactor(1);//Keep in rotaion per minute (ew) by default
            config.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder);
                // .pid(0.0, 0.0, 0.0);
            Motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        }
        
    }
    
    /**
     * Configures motor by calling {@link SparkMax#configure(SparkBaseConfig, ResetMode, PersistMode)}
     * @param configs The configuration
     * @param resetMode I don't know, see {@link SparkMax#configure(SparkBaseConfig, ResetMode, PersistMode)}
     * @param persistMode I don't know, see {@link SparkMax#configure(SparkBaseConfig, ResetMode, PersistMode)}
     * @return {@link REVLibError#kOk} if successful
      */
    public REVLibError configure(SparkMaxConfig configs, ResetMode resetMode, PersistMode persistMode){
       return Motor.configure(configs, resetMode, persistMode);
    }
    
    /**
     * Sets the reference of the {@link #CLController}. Should be called periodically.
     * Ensures the motor has not passed the min or max rotations.
      */
    public void resetReference(){
        //to prevent to motor from turning to far down
        double motPos = Motor.getEncoder().getPosition();
        if(motPos <= minRot){
            refVal = minRot + minRotOffset;
            controlT = ControlType.kPosition;
        }
        else if(motPos >= maxRot){
            refVal = maxRot - maxRotOffset;
            controlT = ControlType.kPosition;
        }
        defaultSetRef();
    }
    
    /**
     * Here in case a more complicated way of setting references is needed, 
     * because accessing {@link #CLController} directly to set reference might be overridden
     * when {@link #resetReference()} is called. In short, this allows the other 
     * {@link SparkClosedLoopController#setReference(double, ControlType)} methods to be used.
     * @see SparkClosedLoopController#setReference(double, ControlType, com.revrobotics.spark.ClosedLoopSlot)
     * @see SparkClosedLoopController#setReference(double, ControlType, com.revrobotics.spark.ClosedLoopSlot, double)
     * @see SparkClosedLoopController#setReference(double, ControlType, com.revrobotics.spark.ClosedLoopSlot, double, com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits)
     * @return The {@code REVLibError} normaly returned by {@link SparkClosedLoopController#setReference(double, ControlType)}
      */
    protected REVLibError defaultSetRef(){
        return CLController.setReference(refVal, controlT);
    }
    
    /**
     * Uses the Closed loop controller to set it's reference.
     * <p>
     * <b>Note:</b> the set reference will be overridden unless {@link #defaultSetRef()}
     * is changed overriden to do nothing.
     * </p>
     * @param value The value to set depending on the control type.
     * @param type The control type.
     * @return The {@code REVLibError} normaly returned by {@link SparkClosedLoopController#setReference(double, ControlType)}
      */
    public REVLibError setRefernce(double value, ControlType type){
        return CLController.setReference(value, type);
    }
    
    /**
     * Gets number of rotaions when given the percent from min to max.
     * <p>
     * (i.e. 1 or 100% returns the {@link #maxRot maximum rotations},
     *  0 or 0% returns the {@link #minRot minimum rotations})
     * </p>
     * @param percent the percent of rotations
     * @return the absolute value of rotations
      */
    public double getRotationsFromPercent(double percent){
        return (maxRot-minRot) *percent+ minRot;
    }
    
    /**
     * Sets speed to value between -1 and 1
     * @param speed the speed as a percent from -1 to 1
      */
    public void setSpeed(double speed){
        controlT = ControlType.kDutyCycle;
        refVal = Math.min(Math.max(speed, -1), 1);
    }
    
    /**
     * Moves the motor by the given number of rotations.
     * @param numberOfRotations the number of rotations to turn by.
      */
    public void moveByRotations(double numberOfRotations){
        double currentRot = Motor.getEncoder().getPosition();
        refVal = Math.min(Math.max(currentRot + numberOfRotations, minRot), maxRot);
        controlT = ControlType.kPosition;
    }
    
    /**
     * Moves motor towords the percent of rotations given.
     * @param percentRotation percent of rotaions, is element of [0, 1]
     * @see #getRotationsFromPercent(double)
      */
    public void goToRotationPercent(double percentRotation){
        refVal = (maxRot-minRot) * Math.min(Math.max(percentRotation, 0), 1) + minRot;
        controlT = ControlType.kPosition;
    }
    
    /**
     * Moves motor towords the number of rotations given.
     * @param rotations the number of rotations, between {@link #maxRot maximum rotations} 
     * and {@link #minRot minimum rotations}.
      */
    public void goToRotation(double rotations){
        refVal = Math.min(Math.max(rotations, minRot), maxRot);
        controlT = ControlType.kPosition;
    }
    
    public void goToStart(){
        goToRotation(0);
    }
    
    public Command goToStartCommand(Subsystem... requirements){
        return Commands.runOnce(this::goToStart, requirements);
    }
    
    public double getMaxRot() {
        return maxRot;
    }

    public RevMotor setMaxRot(double newMaxRot) {
        maxRot = Math.max(newMaxRot, minRot+minRotOffset);
        return this;
    }
    
    public double getMinRot() {
        return minRot;
    }

    public RevMotor setMinRot(double newMinRot) {
        minRot = Math.min(maxRot, newMinRot);
        return this;
    }
    
    public double getMinRotOffset() {
        return minRotOffset;
    }

    public RevMotor setMinRotOffset(double newMinRotOff) {
        minRotOffset = newMinRotOff;
        return this;
    }
    
    public double getMaxRotOffset() {
        return maxRotOffset;
    }

    public RevMotor setMaxRotOffset(double newMaxRotOffset) {
        maxRotOffset = newMaxRotOffset;
        return this;
    }
    
    public static class RevMotorSetPosition extends RevMotor{
        /** The percents of rotation position to move motor to.  */
        final double[] setPositions;
        /**
         * Constructs motor with wanted set positions.
         * @param deviceId The device ID
         * @param type The type of motor
         * @param Positions the percent between {@link #maxRot max rotations} 
         * and {@link #minRot min rotations} wanted per position
          */
        public RevMotorSetPosition(int deviceId, MotorType type, double... Positions){
            super(deviceId, type);
            setPositions = Positions;
        }
        
        /**
         * Uses the motor given and configures if requested.
         * @param motor The motor to use.
         * @param IsAlreadyConfigured Whether the motor is/will be configured
         * @param percentOfPositions the percent between {@link #maxRot max rotations} 
         * and {@link #minRot min rotations} wanted per position
         */
        public RevMotorSetPosition(SparkMax motor, boolean IsAlreadyConfigured, double... Positions){
            super(motor, IsAlreadyConfigured);
            setPositions = Positions;
        }
    
        /**
         * Goes to the set position wanted.
         * @param positionNumber the index of given positions
          */
        public void goToSetPosition(int positionNumber){
            if(positionNumber > setPositions.length || positionNumber < 0) 
                positionNumber=0;
            goToRotation(setPositions[positionNumber]);
        }
    }
}