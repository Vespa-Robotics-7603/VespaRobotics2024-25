package frc.robot.components;

public class DriveTrain {
    SwerveModule module1;
    SwerveModule module2;
    SwerveModule module3;
    SwerveModule module4;
    double turn = 0;
    public DriveTrain(SwerveModule ... modules){
        module1 = modules[0];
        module2 = modules[1];
        module3 = modules[2];
        module4 = modules[3];
    }
    public void setDriveDirection(double angle){
        module1.TurnTo(angle+turn);
        module2.TurnTo(angle+turn);
        module3.TurnTo(angle-turn);
        module4.TurnTo(angle-turn);
    }
    public void setSpeed(double speed){
        module1.setDriveSpeed(speed);
        module2.setDriveSpeed(speed);
        module3.setDriveSpeed(speed);
        module4.setDriveSpeed(speed);
    }
    public void setTurn(double turn) {
        this.turn = turn;
    }
}
