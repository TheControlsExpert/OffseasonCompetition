public class RollersIO {
    

    public class RollerIOInputs {
        //CAN all good?
        public boolean MotorleftIntakeConnected;
        public boolean MotorrightIntakeConnected;
        public boolean MotorHandoffConnected;

        //-1 to 1
        public double setpoint_leftIntake;
        public double setpoint_rightIntake;
        public double setpoint_Handoff;

        //if something wrong happens
        public double current_leftIntake;
        public double current_rightIntake;
        public double current_Handoff;
        
    }


    public RollersIO() {

    }




}