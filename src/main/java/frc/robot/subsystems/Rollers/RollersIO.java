package frc.robot.subsystems.Rollers;

import org.littletonrobotics.junction.AutoLog;

public interface RollersIO{
    @AutoLog
    public static class RollersIOInputs {
        //CAN all good?
        //Edit: no such function for neos :(
        // public boolean MotorleftIntakeConnected;
        // public boolean MotorrightIntakeConnected;
        // public boolean MotorHandoffConnected;

        //-1 to 1
        public double setpoint_leftIntake = 0;
        public double setpoint_rightIntake = 0;
        public double setpoint_Handoff = 0;

        //if something wrong happens
        public double current_leftIntake = 0;
        public double current_rightIntake = 0;
        public double current_Handoff = 0;

        public double temperature_Handoff = 0;
        public double temperature_leftIntake = 0;
        public double temperature_rightIntake = 0;
        
    }

    default void updateInputs(RollersIOInputs inputs) {

    }



    
}
