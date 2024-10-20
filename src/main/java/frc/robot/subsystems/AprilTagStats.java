package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
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
    PhotonCamera arduCam = new PhotonCamera("Arducam_OV9281_USB_Camera (1)");
    
    //Object representation of the field
    AprilTagFieldLayout layout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    
    //Positonal object representation of the camera on the robot
    Transform3d robotToCam = new Transform3d(new Translation3d(cameraTranslationConstants.tX, cameraTranslationConstants.tY, cameraTranslationConstants.tZ), new Rotation3d(cameraRotationConstants.rRoll, cameraRotationConstants.rPitch, cameraRotationConstants.rYaw));
    
    double yaw, pitch, area;
    int id;

    public AprilTagStats() {
        //construction of a new pose estimator (not tested)
        //PhotonPoseEstimator estimator = new PhotonPoseEstimator(layout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, arduCam, robotToCam);
    }

    public void Stats() {
        //check to see if the camera is stablely connected
        if(arduCam.isConnected()) {

            //gets the newest results from the camera
            var result = arduCam.getLatestResult();
            
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

                //if an april tag is spotted, calculates and returns the 3d pose of the robot (robot relative to field including translation and rotation)
                if (layout.getTagPose(target.getFiducialId()).isPresent()) {
                    Pose3d relativePose = relativeRobot(transform, target, robotToCam, layout);
                }
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

    public Pose3d relativeRobot(Transform3d transform, PhotonTrackedTarget target, Transform3d robotToCamera, AprilTagFieldLayout layout) {
        //intakes the space between a camera and its target, the target itself, the cameras position and rotation on the robot, and the field. Outputs a the robot relative to the field.
        return PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(), layout.getTagPose(target.getFiducialId()).get(), robotToCamera);
    }
}