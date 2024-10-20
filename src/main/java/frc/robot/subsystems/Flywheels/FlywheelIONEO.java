package frc.robot.subsystems.Flywheels;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants;

public class FlywheelIONEO implements FlywheelIO {
    private CANSparkMax shooterT = new CANSparkMax(Constants.ShooterConstants.T_ID, MotorType.kBrushless);
    private CANSparkMax shooterB = new CANSparkMax(Constants.ShooterConstants.B_ID, MotorType.kBrushless);

    public FlywheelIONEO() {
        shooterT.getPIDController().setP(Constants.ShooterConstants.kP);
        shooterT.getPIDController().setP(Constants.ShooterConstants.kD);
        shooterT.setInverted(Constants.ShooterConstants.T_Inverted);

        shooterB.getPIDController().setP(Constants.ShooterConstants.kP);
        shooterB.getPIDController().setD(Constants.ShooterConstants.kD);
        shooterB.setInverted(Constants.ShooterConstants.B_Inverted);
    }


    @Override
    public void set_B_vel(double velocity) {
        double feedforward = Constants.ShooterConstants.kS + Constants.ShooterConstants.kV * velocity + Constants.ShooterConstants.kA * velocity;
        shooterB.getPIDController().setReference(velocity, com.revrobotics.CANSparkBase.ControlType.kVelocity, 0, feedforward, SparkPIDController.ArbFFUnits.kVoltage);
    }

    @Override
    public void set_T_vel(double velocity) {
        double feedforward = Constants.ShooterConstants.kS + Constants.ShooterConstants.kV * velocity + Constants.ShooterConstants.kA * velocity;
        shooterT.getPIDController().setReference(velocity, com.revrobotics.CANSparkBase.ControlType.kVelocity, 0, feedforward, SparkPIDController.ArbFFUnits.kVoltage); 
    }


    @Override
    public void updateInputs(FlywheelIOInputs inputs) {
        inputs.Tvelocity = shooterT.getEncoder().getVelocity();
        inputs.Tcurrent = shooterT.getOutputCurrent();
        inputs.Ttemperature = shooterT.getMotorTemperature();
        inputs.Bvelocity = shooterB.getEncoder().getVelocity();
        inputs.Bcurrent = shooterB.getOutputCurrent();
        inputs.Btemperature = shooterB.getMotorTemperature();
        }


    
    
}
