package frc.robot.subsystems;

import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric;

import edu.wpi.first.math.filter.SlewRateLimiter;

public class FieldCentricDrivetrain extends FieldCentric {
    private SlewRateLimiter xfilter = new SlewRateLimiter(10);
    private SlewRateLimiter yfilter = new SlewRateLimiter(10); 
    private int isinverted;
    
    @Override
    public FieldCentric withVelocityX(double targetvelocity) {
        double filteredvelocity = (
            (this.VelocityX < targetvelocity)
            ? xfilter.calculate(targetvelocity)
            : targetvelocity
        );

        this.VelocityX = filteredvelocity * isinverted;
        return this;
    }

    @Override
    public FieldCentric withVelocityY(double targetvelocity) {
        double filteredvelocity = (
            (this.VelocityY < targetvelocity)
            ? yfilter.calculate(targetvelocity)
            : targetvelocity
        );

        this.VelocityY = filteredvelocity * isinverted;
        return this;
    }

    @Override
    public FieldCentric withRotationalRate(double newRotationalRate) {
        this.RotationalRate = newRotationalRate * isinverted;
        return this;
    }

    // TODO: fix this method
    public FieldCentricDrivetrain setInverted(Boolean isinverted) {
        this.isinverted = (isinverted) ? -1 : 1;
        return this;
    }

    public FieldCentricDrivetrain() {
        super();
        isinverted = -1;
    }
}
