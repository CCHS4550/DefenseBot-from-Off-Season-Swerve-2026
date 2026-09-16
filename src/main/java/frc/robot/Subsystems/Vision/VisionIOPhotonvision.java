package frc.robot.Vision;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import frc.robot.Constants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;

public class VisionIOPhotonvision implements VisionIO {
    private PhotonCamera camera;

    private PhotonPoseEstimator poseEstimator;

    private Transform3d cameraOffset;
    

    // other potential things to add - suppliers for pose, heading data, ambiguity 
    // (lowk sounds like something we should talk about)
    

    public VisionIOPhotonvision (String cameraName, Transform3d cameraOffset){
        camera = new PhotonCamera(cameraName);

        this.cameraOffset = cameraOffset;

        // regional events in the USA use welded fields as per the game manual
        poseEstimator = new PhotonPoseEstimator(Constants.VisionConstants.fieldLayout, cameraOffset);

        camera.getAllUnreadResults(); //
    }

    @Override
    public void updateInputs(VisionIOInputs inputs){
        inputs.connected = camera.isConnected();

        // PhotonPipelineResults are objects thaat basically just store EVERYTHING possible that the camera notices at a given point in time
        // for example, in a PPR, it holds what tags the camera is seeing, the ambiguity, and stuff like that, from which you access it using
        // some getters that you can put in other variables
        List<PhotonPipelineResult> photonPipelineResults;

        // PoseObservation is a record that we made in the VisionIO.java class. What a record really is that it holds data for future use
        // In PO, we just store stuff from PPR in a more accesible format. What's cool about records is that instead of making an entirely
        // separate class with getters and setters to access the variables, the "record" does all of that by itself
        List<PoseObservation> observations;

        if (inputs.connected){
            photonPipelineResults = camera.getAllUnreadResults(); // putting all of that data into the list
            observations = new LinkedList<PoseObservation>();

            ArrayList<Integer> tagIds = new ArrayList<>();
            // looping through the PPR to unpack the data
            for (var result : photonPipelineResults){
                var multiTagResult = result.getMultiTagResult();
                
                if (multiTagResult.isPresent()){
                    //var fieldToCamera = multiTagResult.get().estimatedPose.best; // ripped from the docs btw
                    //var fieldToRobot = fieldToCamera.plus(cameraOffset);

                   
                   // there are other ways to do this
                    poseEstimator.estimateCoprocMultiTagPose(result);


                    // Pose3d robotPose = new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation()); <- also valid way to do ts
                    Optional<EstimatedRobotPose> estimatedPose = coprocEstimate(result);  

                    double totalTagDistance = 0.0;
                   
                    for (PhotonTrackedTarget target : result.targets){
                        totalTagDistance += target.bestCameraToTarget.getTranslation().getNorm();
                        tagIds.add(target.fiducialId);
                    }

                    inputs.tagIds = new int[tagIds.size()];

                    for (int i = 0; i < tagIds.size(); i++){
                        inputs.tagIds[i] =(Integer) tagIds.toArray()[i];
                    }

                    observations.add(
                        new PoseObservation(
                            estimatedPose.get().timestampSeconds,
                            estimatedPose.get().estimatedPose,
                            result.multitagResult.get().estimatedPose.ambiguity,
                            result.multitagResult.get().fiducialIDsUsed.size(),
                            totalTagDistance / result.targets.size(),
                            result.getBestTarget().getYaw(),
                            result.getBestTarget().getPitch(),
                            PoseObservationType.PHOTONVISION_MULTI_TAG
                        )
                    );

                }

                else {
                    // shoul 
                    //var target = result.targets.get(0);

                    //for loop only loops once so it's kinda useless
                    for (PhotonTrackedTarget target : result.targets) {
                    var tagPose = Constants.VisionConstants.fieldLayout.getTagPose(target.fiducialId);
                    if (tagPose.isPresent()){
                        Transform3d fieldToTarget = new Transform3d(tagPose.get().getTranslation(), tagPose.get().getRotation());
                        Transform3d cameraToTarget = target.bestCameraToTarget;
                        Transform3d fieldToCamera = fieldToTarget.plus(cameraToTarget.inverse());
                        Transform3d fieldToRobot = fieldToCamera.plus(cameraOffset.inverse());
                        Pose3d robotPose = new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());

                        tagIds.add(target.fiducialId);

                        observations.add(
                        new PoseObservation(
                            result.getTimestampSeconds(),
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
        }
        
        inputs.poseObservations = observations.toArray(new PoseObservation[observations.size()]);
        
        }
    }

    @Override
    public void applyCameraTransformation(Transform3d transformation) {
        cameraOffset = transformation;
        poseEstimator.setRobotToCameraTransform(transformation);
    }

    @Override
    public void setReferencePose(Pose2d resetPose){

    }

    // if any errors, can correct with this
    @Override
    public void setLastPose(Pose2d lastPose) {

    }

    public Optional<EstimatedRobotPose> coprocEstimate (PhotonPipelineResult result){
        // generally the most accurate way for estimation
        // "combines all visible tag corners"
        return poseEstimator.estimateCoprocMultiTagPose(result);
    }

    public Optional<EstimatedRobotPose> lowestAmbiguityEstimate(PhotonPipelineResult result){
        return poseEstimator.estimateLowestAmbiguityPose(result);
    }
}