public class Vision {
    
}
package frc.robot.Vision;

import java.util.ArrayList;
import java.util.List;

import com.google.flatbuffers.Constants;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Vision.VisionIO.PoseObservation;
import frc.robot.Vision.VisionIO.VisionIOInputs;
import frc.robot.Constants.VisionConstants;

public class Vision extends SubsystemBase {

    private final VisionIO[] io;

    private final VisionIOInputsAutoLogged[] inputs;

    private final Alert[] disconnectedAlerts;

    public final List<List<PoseObservation>> observations;

    public Vision(VisionIO... io) {
        this.io = io;

        this.inputs = new VisionIOInputsAutoLogged[io.length];
        this.disconnectedAlerts = new Alert[io.length];

        for (int i = 0; i < io.length; i++) {
            inputs[i] = new VisionIOInputsAutoLogged();

            disconnectedAlerts[i] = new Alert("Camera " + i + " disconnected", AlertType.kWarning);
        }
    }

    @Override
    public void periodic() {
        for (int i = 0; i < io.length; i++) {
            io[i].updateInputs(inputs[i]);
        }

        for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
            disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected);

            observations.add(new ArrayList<PoseObservation>());

            for (int i = 0; i < inputs[cameraIndex].length; i++) {
                PoseObservation observation = inputs[cameraIndex].poseObservations[i];

                //Check for bad observation
                if(observation.tagCount() == 0 // Must have at least one tag
                        || observation.ambiguity() > VisionConstants.maxAmbiguity // Cannot be high ambiguity
                        || Math.abs(observation.pose().getZ()) > VisionConstants.maxZError // Must have realistic Z coordinate
                        || observation.averageTagDistance() > 5.25

                        // Must be within the field boundaries
                        // || observation.pose().getX() < 0.0
                        // || observation.pose().getX() > FieldConstants.FIELD_LAYOUT.getFieldLength()
                        // || observation.pose().getY() < 0.0
                        // || observation.pose().getY() > FieldConstants.FIELD_LAYOUT.getFieldWidth()
                        ) 
                {
                    continue;
                }
                
                observations.get(i).add(observation);
            }
        }
    }
}