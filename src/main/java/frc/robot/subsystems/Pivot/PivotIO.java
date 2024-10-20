package frc.robot.subsystems.Pivot;

import org.littletonrobotics.junction.AutoLog;

public interface PivotIO {
    @AutoLog
    public static class PivotIOInputs {
        public double encoderpos = 0;
        public double encodervel = 0;
        public double current = 0;
        public double temperature = 0;
    }

    default void updateInputs(PivotIOInputs inputs) {}

    default void setSetpointInRotations(double setpointInRotations) {
        
    }  

    

    
}  
