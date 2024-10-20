package frc.robot.subsystems.Vision.Limelight3G;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;

public class Vision_3g_Subsystem extends SubsystemBase {

        
    
        public Vision_3GF_IOInputsAutoLogged inputs = new Vision_3GF_IOInputsAutoLogged();
        public Vision_3GF_IO_class vision3gIO;
        public Vision_3g_Subsystem(Vision_3GF_IO_class vision3gIO) {
            this.vision3gIO = vision3gIO;
    
        }
    
        @Override
        public void periodic() {
            vision3gIO.updateInputs(inputs);

            if (getPosell3gf().timestamp > getPosell3gs().timestamp) {
        
            RobotState.publishVision(getPosell3gs());
            RobotState.publishVision(getPosell3gf());
            }
            else {
            RobotState.publishVision(getPosell3gf());
            RobotState.publishVision(getPosell3gs());
            }
        }    

        public void set(int nf, int ns) {
            Logger.recordOutput("nf leds", nf);
            Logger.recordOutput("ns leds", ns);
            vision3gIO.setLEDsll3gf(nf);
            vision3gIO.setLEDSll3gs(ns);
        }


        
        public PoseInfo getPosell3gf() {
            return new PoseInfo(inputs.ll3gf_pose, inputs.timestamp_ll3gf, inputs.avgDistance_3gf, inputs.avgTagAreaf, inputs.ignoreUpdatef);
        }

        public PoseInfo getPosell3gs() {
            return new PoseInfo(inputs.ll3gs_pose, inputs.timestamp_ll3gs, inputs.avgDistance_3gs, inputs.avgTagAreas, inputs.ignoreUpdates);
        }

        public double getSpeakerDetection() {
            return inputs.tx;
        }



        public record PoseInfo(Pose2d pose, double timestamp, double avgDistance, double avgTagArea, boolean ignoreUpdate) {}



    
        

    
    
}
