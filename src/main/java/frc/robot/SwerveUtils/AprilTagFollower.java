package frc.robot.SwerveUtils;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.function.BooleanSupplier;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.subsystems.TrajectoryFollower;
import frc.robot.subsystems.Vision;

public class AprilTagFollower {
    
    TrajectoryFollower follower;
    Vision photonVison;
    
    //This is so that we can stop reading/moving to april tags
    private boolean stopFollowing = false;
    
    public AprilTagFollower(Vision photonVision, TrajectoryFollower follower){
        this.follower = follower;
        this.photonVison = photonVision;
    }
    
    public Command makeFollowCommand(double XCoord, double YCoord, double Yaw){
        return follower.wrapGeneratedCommand( new TrajectoryTarget2d(XCoord, YCoord, Yaw));
    }
    
    @Deprecated
    public PhotonTrackedTarget scanFor(int id){
        List<PhotonTrackedTarget> targets = photonVison.getAllResults();
        for (PhotonTrackedTarget target : targets) {
            if(target.getFiducialId() == id) return target;
        }
        return null;
    }
    
    public void followMultipleTags(Map<Integer, FollowTagData> data){
        // List<PhotonTrackedTarget> targets = photonVison.getAllResults();
        // System.out.println("getting results");
        List<PhotonTrackedTarget> topTargets = photonVison.getAllTopResults();
        // System.out.println("Got them");
        List<PhotonTrackedTarget> bottomTargets = photonVison.getAllBottomResults();
        List<PhotonTrackedTarget> targets = new ArrayList<>(bottomTargets);
        targets.addAll(topTargets);
        for (PhotonTrackedTarget target : topTargets) {
            if(!stopFollowing && data.containsKey(target.getFiducialId())){
                //run schedule command maker
                scheduleFollowCommand(data.get(target.getFiducialId()), target);
                return;
            };
        }
    }
    
    public void scheduleFollowCommand(FollowTagData data, PhotonTrackedTarget target){
        if(data == null){
            System.out.println("Map getting error, or null given");
            return;
        }
        
        Transform3d location = target.getBestCameraToTarget();
        
        // TODO: decide if positive offsets should move the robot forward
        // or backward, decide upon whether or not the subtraction of
        // offsets here should also apply for side-to-side movements.
        Command followCommand = makeFollowCommand(
            location.getX() - data.offsetX,
            location.getY() - data.offsetY,
            location.getRotation().getZ() - data.offsetYaw
        );
        // both commands will run until one finishes, and waitUntil ends on command
        Command cancel = Commands.race(
            Commands.waitUntil(data.cancelCondition),
            followCommand
        );
        
        CommandScheduler.getInstance().schedule(cancel); 
        // we don't want to repeatedly schedule the same command
        // or multiple follow commands
        stopFollowing = true;
        System.out.println("SCHEDULED!");
    }
    
    @Deprecated
    public void scheduleFollowCommand(int id, double offsetX, double offsetY, double offsetRAD, BooleanSupplier cancelCondition){
        PhotonTrackedTarget target = scanFor(id);
        if(target != null && !stopFollowing){
            //found, now follow
            Transform3d location = target.getBestCameraToTarget();
            
            Command followCommand = makeFollowCommand(
                location.getX() + offsetX,
                location.getY() + offsetY,
                location.getRotation().getZ() + offsetRAD
            );
            // both commands will run until one finishes, and waitUntil ends on command
            Command cancel = Commands.race(
                Commands.waitUntil(cancelCondition),
                followCommand
            );
            
            CommandScheduler.getInstance().schedule(cancel); 
            // we don't want to repeatedly schedule the same command
            // or multiple follow commands
            stopFollowing = true;
            System.out.println("SCHEDULED!");
        }
    }
    
    public Command alignWithTag(int seconds, FollowTagData... tags){
        Map<Integer, FollowTagData> tagsToSearchFor = new LinkedHashMap<>();
        for (FollowTagData data : tags) {
            tagsToSearchFor.put(data.id, data);
        }
        return Commands.race(
            Commands.waitSeconds(seconds),
            // run until given time is up
            Commands.runEnd(
                ()->followMultipleTags(tagsToSearchFor),
                ()->stopFollowing = false,
                photonVison
            )
        ).andThen(Commands.print("ALIGNED... I think"));
    }
    
    public Command alignWithTag(double seconds, int id, double offsetX, double offsetY, double offsetRAD, BooleanSupplier cancelCondition){
        
        // return Commands.race(
        //     Commands.waitSeconds(seconds),
        //     // run until given time is up
        //     Commands.runEnd(
        //         ()->scheduleFollowCommand(id, offsetX, offsetY, offsetRAD, cancelCondition),
        //         ()->stopFollowing = false,
        //         photonVison
        //     )
        // ).andThen(Commands.print("ALIGNED... I think"));
        return alignWithTag(id, new FollowTagData(id, offsetX, offsetY, offsetRAD, cancelCondition));
    }
    
    public Command alignWithTag(int id, double offsetX, double offsetY, double offsetRAD, BooleanSupplier cancelCondition){
        return alignWithTag(1, id, offsetX, offsetY, offsetRAD, cancelCondition);
    }
    
    public Command alignWithTag(double seconds, int id, double offsetX, double offsetY, double offsetRAD){
        return alignWithTag(seconds, id, offsetX, offsetY, offsetRAD, ()->false);
    }
    
    public Command alignWithTag(int id, double offsetX, double offsetY, double offsetRAD){
        return alignWithTag(id, offsetX, offsetY, offsetRAD, ()->false);
    }
    
    public Command alignWithTag(int id){
        return alignWithTag(id, 0, 0, 0);
    }
    
    public static class FollowTagData{
        int id;
        double offsetX;
        double offsetY;
        double offsetYaw;
        BooleanSupplier cancelCondition;
        public FollowTagData(int id, double offsetX, double offsetY, double offsetYaw, BooleanSupplier cancelCondition) {
            this.id = id;
            this.offsetX = offsetX;
            this.offsetY = offsetY;
            this.offsetYaw = offsetYaw;
            this.cancelCondition = cancelCondition;
        }
    }


}
