package frc.robot.Vision;

import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Optional;
import java.util.Set;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.Constants.VisionConstants;

public class VisionIOPhotonSim implements VisionIO {
    private PhotonCameraSim sim;

    private VisionSystemSim visionSystemSim;

    private PhotonPoseEstimator photonPoseEstimator;

    public Matrix<N3, N1> stddev;

    // supplier?

    public VisionIOPhotonSim(String cameraName, Transform3d cameraTransform) {
        sim = new PhotonCameraSim(new PhotonCamera(cameraName));
        sim.setMaxSightRange(VisionConstants.kCameraMaxRange);
        sim.enableProcessedStream(true);
        visionSystemSim = new VisionSystemSim("Talon Cam" + cameraName);
        visionSystemSim.addAprilTags(VisionConstants.fieldLayout);
        visionSystemSim.addCamera(sim, cameraTransform);
        this.photonPoseEstimator = new PhotonPoseEstimator(
                VisionConstants.fieldLayout, PoseStrategy.CONSTRAINED_SOLVEPNP, cameraTransform);
        photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.PNP_DISTANCE_TRIG_SOLVE);
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        inputs.connected = sim.getCamera().isConnected();
        List<PhotonPipelineResult> results = sim.getCamera().getAllUnreadResults();
        List<PoseObservation> obs = new LinkedList<PoseObservation>();

        for (int i = results.size() - 1; i >= 0; i--) {
            if (results.get(i).multitagResult.isPresent()) {
                Optional<EstimatedRobotPose> estimatedPose = photonPoseEstimator.update(results.get(i));
                double totalTagDistance = 0.0;
                Set<Short> tagIdSet = new HashSet<>();
                for (PhotonTrackedTarget target : results.get(i).targets) {
                    totalTagDistance += target.bestCameraToTarget.getTranslation().getNorm();
                    tagIdSet.add((short) target.fiducialId);
                }
                inputs.tagIds = new int[tagIdSet.size()];
                for (int j = 0; j < tagIdSet.size(); j++) {
                    inputs.tagIds[j] = (Short) tagIdSet.toArray()[j];

                }

                obs.add(
                        new PoseObservation(
                                estimatedPose.get().timestampSeconds,
                                estimatedPose.get().estimatedPose,
                                results.get(i).multitagResult.get().estimatedPose.ambiguity,
                                results.get(i).multitagResult.get().fiducialIDsUsed.size(),
                                totalTagDistance / results.get(i).targets.size(),
                                results.get(i).getBestTarget().getYaw(),
                                results.get(i).getBestTarget().getPitch(),
                                PoseObservationType.PHOTONVISION_MULTI_TAG));

            } else {
                List<PhotonTrackedTarget> targets = results.get(i).getTargets();
                Optional<EstimatedRobotPose> pose = photonPoseEstimator.update(results.get(i));

                for (PhotonTrackedTarget target : targets) {
                    Pose3d robotPose = new Pose3d();
                    if (VisionConstants.fieldLayout.getTagPose(target.getFiducialId()).isPresent()) {
                        robotPose = PhotonUtils.estimateFieldToRobotAprilTag(
                                target.getBestCameraToTarget(),
                                VisionConstants.fieldLayout.getTagPose(target.getFiducialId()).get(),
                                photonPoseEstimator.getRobotToCameraTransform().inverse());

                    }
                    obs.add(
                            new PoseObservation(
                                    pose.get().timestampSeconds,
                                    robotPose,
                                    target.getPoseAmbiguity(),
                                    1,
                                    target.getBestCameraToTarget().getTranslation().getNorm(),
                                    target.getYaw(),
                                    target.getPitch(),
                                    PoseObservationType.PHOTONVISION_SINGLE_TAG));

                }
            }
        }
        inputs.poseObservations = obs.toArray(new PoseObservation[0]);
    }
}