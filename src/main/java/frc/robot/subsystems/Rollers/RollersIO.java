package frc.robot.subsystems.Rollers;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

public class RollersIO {
    CANSparkMax intakeleft = new CANSparkMax(Constants.IntakeConstants.LEFTINTAKE_ID, MotorType.kBrushless);
    CANSparkMax intakeright = new CANSparkMax(Constants.IntakeConstants.RIGHTINTAKE_ID, MotorType.kBrushless);
    CANSparkMax handoff = new CANSparkMax(Constants.IntakeConstants.HANDOFF_ID, MotorType.kBrushless);

    RollerIOInputs inputs = new RollerIOInputs();



    
    
    
    public class RollerIOInputs {
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
        public double current_leftIntake;
        public double current_rightIntake;
        public double current_Handoff;
        
    }


    public RollersIO() {
        //positive voltage percentage -> note going in
        intakeleft.setInverted(IntakeConstants.LEFT_INVERSION);
        intakeright.setInverted(IntakeConstants.RIGHT_INVERSION);
        handoff.setInverted(IntakeConstants.HANDOFF_INVERSION);
        
        //fuses are rated for 40
        intakeleft.setSmartCurrentLimit(40);
        intakeright.setSmartCurrentLimit(40);
        handoff.setSmartCurrentLimit(40);

        inputs.current_Handoff = handoff.getOutputCurrent();
        inputs.current_leftIntake = intakeleft.getOutputCurrent();
        inputs.current_rightIntake = intakeright.getOutputCurrent();
    }


    public void updateInputs() {
        
    }




}