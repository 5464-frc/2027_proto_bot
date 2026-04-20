package frc.robot.Subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.SmartLogger.loggingItem;
import frc.robot.Subsystems.Vision.collectiveCamera;

public class Vision extends SubsystemBase {
    public static final double OLDEST_POSE = 0.04;
    public static final AprilTagFieldLayout kTagField = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    public Pose3d visionPoseRAW = Pose3d.kZero;
    public Field2d visionField = new Field2d();
    public class collectiveCamera {
        PhotonCamera cam;
        PhotonPoseEstimator estimator;
        Double oldestPoseSeconds;
        List<PhotonPipelineResult> resultCache = new ArrayList<>();
        Optional<EstimatedRobotPose> pose;
        private List<EstimatedRobotPose> poseCache = new ArrayList<>();
        List<EstimatedRobotPose> poseBuffer = new ArrayList<>();

        loggingItem cameraLogs;

        collectiveCamera(String name, PhotonPoseEstimator estimator, Double oldestPoseSeconds) {
            this.cam = new PhotonCamera(name);
            this.cam.setFPSLimit(50); // prevent buildup
            this.estimator = estimator;
            this.oldestPoseSeconds = oldestPoseSeconds;
            this.cameraLogs = new loggingItem(name, 2);
        }

        public void update() {
            poseCache = poseBuffer;
            resultCache = this.cam.getAllUnreadResults();

            if (resultCache.size() > 1) {
                System.err.println("cache buildup (check 20ms threshhold)");
            } else if (resultCache.isEmpty()) {
                return;
            }

            if (!resultCache.get(0).hasTargets()) {
                return;
            }

            this.pose = this.estimator.estimateCoprocMultiTagPose(resultCache.get(0));

            if (this.pose.isEmpty()) {
                // backup method
                this.pose = this.estimator.estimateAverageBestTargetsPose(resultCache.get(0));
                if (this.pose.isEmpty()) {
                    return;
                }
            }
            cameraLogs.pushValue(this.pose.get().toString());
            // add pose to cache
            this.poseCache.add(this.pose.get());

            this.poseBuffer.clear();
            // if pose is too old, remove from cache
            for (int i = 0; i < this.poseCache.size(); i++) {
                if (this.poseCache.get(i).timestampSeconds < this.oldestPoseSeconds) {
                    this.poseBuffer.add(this.poseCache.get(i));
                }
            }
        }
    }

    private PhotonPoseEstimator quickEstimator(Transform3d cameraToCenter) {
        return new PhotonPoseEstimator(kTagField, cameraToCenter);
    }

    collectiveCamera[] cameras = {
            new collectiveCamera("fuzz", quickEstimator(Transform3d.kZero), OLDEST_POSE)
    };

    public Pose3d compiledVisionPose(collectiveCamera[] cameraArray) {
        List<EstimatedRobotPose> compiledPose = new ArrayList<>();
        Rotation3d compiledRotation = Rotation3d.kZero;
        Translation3d compiledTranslation = Translation3d.kZero;
        
        for (collectiveCamera cam : cameraArray) {
            compiledPose.addAll(cam.poseBuffer);
        }

        for (int i = 0; i < compiledPose.size(); i++) {
            compiledRotation.plus(compiledPose.get(i).estimatedPose.getRotation());
            compiledTranslation.plus(compiledPose.get(i).estimatedPose.getTranslation());
        }
        compiledRotation = compiledRotation.div(compiledPose.size());
        compiledTranslation = compiledTranslation.div(compiledPose.size());

        return new Pose3d(compiledTranslation, compiledRotation);
    }

    @Override
    public void periodic() {
        for (collectiveCamera c : cameras) {
            c.update();
            c.cameraLogs.smartdashboardHook();
        }
        visionField.setRobotPose(compiledVisionPose(cameras).toPose2d());
        SmartDashboard.putData("vision_est_field",visionField);
    }
}
