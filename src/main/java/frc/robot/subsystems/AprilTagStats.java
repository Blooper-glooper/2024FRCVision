package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.cameraRotationConstants;
import frc.robot.Constants.cameraTranslationConstants;

public class AprilTagStats extends SubsystemBase {
    //Creating new object for the arducam
    private final PhotonCamera m_arduCam = new PhotonCamera("Arducam_OV9281_USB_Camera (1)");
    
    //Object representation of the field
    AprilTagFieldLayout m_layout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    
    //Positional object representation of the camera on the robot
    private final Transform3d m_robotToCam = new Transform3d(new Translation3d(cameraTranslationConstants.tX, cameraTranslationConstants.tY, cameraTranslationConstants.tZ), new Rotation3d(cameraRotationConstants.rRoll, cameraRotationConstants.rPitch, cameraRotationConstants.rYaw));
    
    //Photon Pose Estimator object
    PhotonPoseEstimator m_photonPoseEstimator;

    //Shuffleboard tab named vision, and 4 different widgets for yaw, pitch, tag id, and distance to a tag
    private ShuffleboardTab m_tab = Shuffleboard.getTab("Vision");
    private GenericEntry m_yawEntry, m_pitchEntry, m_idEntry, m_distanceEntry;

    //Variables to hold all of the widget values
    private double m_yaw, m_pitch, m_distance;
    private int m_id;

    //Generic starting position of the robot
    private final Pose3d m_startPose3d = new Pose3d(12.5, 5.5, 0, new Rotation3d());

    StructPublisher<Pose3d> m_publisher;

    public AprilTagStats() {
        m_yawEntry = m_tab.add("yaw", m_yaw).getEntry();
        m_pitchEntry = m_tab.add("pitch", m_pitch).getEntry();
        m_idEntry = m_tab.add("id", m_id).getEntry();
        m_distanceEntry = m_tab.add("distance", m_distance).getEntry();
        
        m_publisher = NetworkTableInstance.getDefault().getStructTopic("Current Robot Pose", Pose3d.struct).publish();

        m_photonPoseEstimator = new PhotonPoseEstimator(m_layout, PoseStrategy.CLOSEST_TO_LAST_POSE, m_arduCam, m_robotToCam);
        updatePose(m_startPose3d);
    }

    public void Stats() {
        //check to see if the camera is stably connected
        if (m_arduCam.isConnected()) {

            //gets the newest results from the camera
            var result = m_arduCam.getLatestResult();
            
            if (result.hasTargets()) {

                //gets the best target available when the camera has results
                PhotonTrackedTarget target = result.getBestTarget();

                //pulls out yaw, pitch, and id of the located apriltag
                m_yaw = target.getYaw();
                m_pitch = target.getPitch();
                m_id = target.getFiducialId();

                //gets the transformation that maps the space between the camera and the tag 
                Transform3d transform = target.getBestCameraToTarget();

                //if an april tag is spotted, calculates and returns the 3d pose of the robot (robot relative to field including translation and rotation)
                if (m_layout.getTagPose(target.getFiducialId()).isPresent()) {
                    Pose3d relativePose = getRobotPose(transform, target, m_robotToCam);
                    updatePose(relativePose);
                    m_distance = getDistance(relativePose, target.getFiducialId());
                }
                updateView(m_yaw, m_pitch, m_id, m_distance);
            }
        } else {
            System.out.println("Not Connected");
        }
    }

    public void updateView(double yaw, double pitch, int id, double distance) {
        //pushes yaw, pitch, id, and distance between the robot and tag to ShuffleBoard (Meant for testing if values are being passed to variables)
        m_yawEntry.setDouble(yaw);
        m_pitchEntry.setDouble(pitch);
        m_idEntry.setInteger(id);
        m_distanceEntry.setDouble(distance);
    }

    public Pose3d getRobotPose(Transform3d transform, PhotonTrackedTarget target, Transform3d robotToCamera) {
        //intakes the space between a camera and its target, the target itself, the camera's position and rotation on the robot, and the field. Outputs the robot relative to the field.
        return PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(), m_layout.getTagPose(target.getFiducialId()).get(), robotToCamera);
    }

    public void updatePose(Pose3d newPose) {
        //Updates the robots pose on network tables
        m_publisher.set(newPose);
    }

    public double getDistance(Pose3d robotPose, int tagID) {
        //Gets the distance between the robot and the apriltag its viewing
        return PhotonUtils.getDistanceToPose(robotPose.toPose2d(), m_layout.getTagPose(tagID).get().toPose2d());
    }
}
