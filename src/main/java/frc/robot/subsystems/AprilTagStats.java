package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import frc.robot.Constants;
import frc.robot.Constants.cameraRotationConstants;
import frc.robot.Constants.cameraTranslationConstants;

public class AprilTagStats extends SubsystemBase{
    //Creating new object for the arducam
    PhotonCamera cam1 = new PhotonCamera("Arducam_OV9281_USB_Camera (1)");
    
    //Object representation of the field
    AprilTagFieldLayout layout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    
    //Positonal object representation of your camera on the robot
    Transform3d robotToCam = new Transform3d(new Translation3d(cameraTranslationConstants.tX, cameraTranslationConstants.tY, cameraTranslationConstants.tZ), new Rotation3d(cameraRotationConstants.rRoll, cameraRotationConstants.rPitch, cameraRotationConstants.rYaw));
    
    double yaw, pitch, area;
    int id;

    public AprilTagStats() {
        //construction of a new pose estimator (not tested)
        //PhotonPoseEstimator estimator = new PhotonPoseEstimator(layout, PoseStrategy.CLOSEST_TO_LAST_POSE, cam1 ,robotToCam);
    }

    public void Stats() {
        //check to see if the camera is stablely connected
        if(cam1.isConnected()) {

            //gets the newest results from the camera
            var result = cam1.getLatestResult();
            
            if (result.hasTargets()) {

                //gets the best target avaliable when the camera has results
                PhotonTrackedTarget target = result.getBestTarget();

                //pulls out yaw, pitch, area, and id of the located apriltag
                yaw = target.getYaw();
                pitch = target.getPitch();
                area = target.getArea();
                id = target.getFiducialId();

                //pushes values to shuffleboard
                updateView(yaw, pitch, area, id);

                //gets the transformation that maps the space between the camera and the tag 
                Transform3d transform = target.getBestCameraToTarget();

            }
            System.out.println("connected");
        }
        else {
            System.out.println("Not Connected");
        }
        //updateView(yaw, pitch, area, id);
    }

    public void updateView(double yaw, double pitch, double area, double id) {
        //pushes yaw, pitch, area, and id to ShuffleBoard (Meant for testing if values are being passed to variables)
        Shuffleboard.getTab("Vision").add("yaw", yaw);
        Shuffleboard.getTab("Vision").add("pitch", pitch);
        Shuffleboard.getTab("Vision").add("area", area);
        Shuffleboard.getTab("Vision").add("id", id);
    }

}
