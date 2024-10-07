package frc.robot.subsystems.Pivot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;


import frc.robot.Constants.PivotConstants;

public class PivotIONEO implements PivotIO {
    private CANSparkMax Pivot;
    


    public PivotIONEO() {
        Pivot = new CANSparkMax(PivotConstants.PivotID, MotorType.kBrushless);

        Pivot.getPIDController().setP(PivotConstants.kP);
        Pivot.getPIDController().setD(PivotConstants.kD);

        Pivot.getEncoder().setPosition(0);
        // 0 at start, increasing from downwards
        Pivot.setInverted(PivotConstants.PivotInversion);
        

        
        



    }

    @Override
    public void updateInputs(PivotIOInputs inputs) {
        inputs.encoderpos = Pivot.getEncoder().getPosition();
        inputs.encodervel = Pivot.getEncoder().getVelocity();
        inputs.current = Pivot.getOutputCurrent();
        inputs.temperature = Pivot.getMotorTemperature();
        
    }

    @Override
    public void setSetpointInRotations(double setpointInRotations) {
        Pivot.getPIDController().setReference(setpointInRotations, ControlType.kPosition);
    }

    
        
}
