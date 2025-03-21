package frc.robot.commands;

import frc.robot.subsystems.Vision;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;

import com.ctre.phoenix6.swerve.SwerveRequest;


public class APT extends Command {
    Timer time = new Timer();
    public APT() {

    }

    @Override
    public void execute(){
        time.start();

        // Initialize PhotonCamera
        PhotonCamera camera;
        camera = new PhotonCamera("topCamera");
        PhotonPipelineResult result = camera.getLatestResult();

        boolean SwitchCam = false;
        SwitchCam = (!result.hasTargets()) ? true :  false;


    }

}



