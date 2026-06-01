package frc.robot.Subsystems;

import java.util.ArrayList;
import java.util.Collections;
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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {

    // only used for calculations / pose averaging
    // not worth using in a majority of other cases
    private class barebonesPose {
        double x, y, z, rx, ry, rz;

        public barebonesPose(double x, double y, double z, double rx, double ry, double rz) {
            this.x = x;
            this.y = y;
            this.z = z;
            this.rx = rx;
            this.ry = ry;
            this.rz = rz;
        }
    }

    private class cameraFrame {
        PhotonPoseEstimator estimator;
        PhotonCamera cameraObject;

        public cameraFrame(String name, PhotonPoseEstimator estimator) {
            this.estimator = estimator;
            this.cameraObject = new PhotonCamera(name);
            if (this.cameraObject.getFPSLimit() > 50) {
                // prevents cache buildup
                // if it's higher then 50, you'd get more frames from the camera
                // then wpilib does frame / second
                // wpilib periodic cycles max at 20ms (before it gets mad at you)
                // 20ms*50 == 1000ms (or, 1 second)

                this.cameraObject.setFPSLimit(50);
            }
        }
    }

    public class DIVA {
        // Decay Implemented Vision Array

        int maxCacheLen; // rec: cam count * like 4-ish
        double oldestPoseSec; // smoothes things out at the cost of slower updates

        cameraFrame[] cameras;
        List<EstimatedRobotPose> resultCache = new ArrayList<>();

        Pose3d calculatedRobotPose = Pose3d.kZero;

        public DIVA(cameraFrame[] cameras, int maxCacheLen, double oldestPoseSec) {
            this.cameras = cameras;
            this.maxCacheLen = maxCacheLen;
            this.oldestPoseSec = oldestPoseSec;
        }

        public void capture() {
            List<PhotonPipelineResult> latestResult;
            for (cameraFrame c : cameras) {

                // needs to be held here to prevent double fetching for check
                Optional<EstimatedRobotPose> cacheItem = Optional.empty();
                latestResult = c.cameraObject.getAllUnreadResults();

                if (latestResult.isEmpty()) {
                    continue;
                }

                if (latestResult.size() > 1) {
                    System.err.println("result buildup on cam " + c.cameraObject.getName());
                }

                // primary method (best)
                cacheItem = c.estimator.estimateCoprocMultiTagPose(latestResult.get(0));
                if (cacheItem.isEmpty()){
                    // backup (works with if only 1 apriltag)
                    cacheItem = c.estimator.estimateAverageBestTargetsPose(latestResult.get(0));
                }
                

                if (cacheItem.isPresent()) {
                    resultCache.add(cacheItem.get());
                }

            }
        }

        public void prune() {
            while (resultCache.size() > maxCacheLen) {
                resultCache.remove(0);
            }

            // compile to list before removing to prevent list changes in for loop (bad
            // practice)
            List<Integer> toPrune = new ArrayList<>();

            // toPrune in order of 0st index - highest
            for (int i = 0; i < resultCache.size(); i++) {
                if (Timer.getTimestamp() - resultCache.get(i).timestampSeconds > oldestPoseSec) {
                    toPrune.add(i);
                }
            }
            if (toPrune.isEmpty()) {
                return;
            }

            Collections.reverse(toPrune);
            for (int i : toPrune) {
                resultCache.remove(i);
            }

        }

        public void calculatePose() {
            if (resultCache.size() == 0) {
                calculatedRobotPose = Pose3d.kZero;
                return;
            }
            List<barebonesPose> aPoses = new ArrayList<>();
            barebonesPose averagedPose = new barebonesPose(0, 0, 0, 0, 0, 0);
            for (EstimatedRobotPose i : resultCache) {
                Pose3d pullFrom = i.estimatedPose;

                aPoses.add(new barebonesPose(pullFrom.getX(), pullFrom.getY(), pullFrom.getZ(),
                        pullFrom.getRotation().getX(), pullFrom.getRotation().getY(), pullFrom.getRotation().getZ()));
            }
            for (barebonesPose i : aPoses) {
                averagedPose.x += i.x;
                averagedPose.y += i.y;
                averagedPose.z += i.z;
                averagedPose.rx += i.rx;
                averagedPose.ry += i.ry;
                averagedPose.rz += i.rz;
            }
            averagedPose.x /= aPoses.size();
            averagedPose.y /= aPoses.size();
            averagedPose.z /= aPoses.size();
            averagedPose.rx /= aPoses.size();
            averagedPose.ry /= aPoses.size();
            averagedPose.rz /= aPoses.size();

            calculatedRobotPose = new Pose3d(averagedPose.x, averagedPose.y, averagedPose.z,
                    new Rotation3d(averagedPose.rx, averagedPose.ry, averagedPose.rz));
        }
    }

    cameraFrame[] cfList = {
            new cameraFrame("barry",
                    new PhotonPoseEstimator(AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField),
                            Transform3d.kZero)),
            new cameraFrame("bar", new PhotonPoseEstimator(AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField),
                    Transform3d.kZero)) };
    DIVA main = new DIVA(cfList, 20, 2);

    Field2d printField2d = new Field2d();

    @Override
    public void periodic() {
        main.capture();
        main.prune();
        main.calculatePose();

        printField2d.setRobotPose(main.calculatedRobotPose.toPose2d());

        SmartDashboard.putData("SDVA field", printField2d);
    }
}