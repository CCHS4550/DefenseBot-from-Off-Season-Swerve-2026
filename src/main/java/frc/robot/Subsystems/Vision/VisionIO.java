package frc.robot.Vision;
import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;

public interface VisionIO {
    
    @AutoLog
    public static class VisionIOInputs {
        public boolean connected = false;
        public PoseObservation[] poseObservations = new PoseObservation[0];
        public int[] tagIds = new int[0];
    }

    public static record PoseObservation (
        double timestamp,
        Pose3d pose, // Pose2d
        double ambiguity,
        int tagCount,
        double averageTagDistance,
        double tx,
        double ty,
        PoseObservationType type
    ){}

    public enum PoseObservationType {
        MEGATAG_1,
        MEGATAG_2,
        PHOTONVISION_MULTI_TAG,
        PHOTONVISION_SINGLE_TAG,
        QUEST_NAV,
        NONE
    }

    public default void updateInputs (VisionIOInputs inputs){}

    // used to set the starting pose for the quest or limelight (more pertinent for quest)
    public default void setReferencePose(Pose2d resetPose){}

    // if any errors, can correct with this
    public default void setLastPose(Pose2d lastPose){}

    public default void applyCameraTransformation(Transform3d transformation){}
}