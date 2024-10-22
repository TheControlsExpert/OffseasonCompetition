package frc.robot.subsystems.Flywheels;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants;

public class FlywheelIONEO implements FlywheelIO {
    private CANSparkMax shooterT = new CANSparkMax(15, MotorType.kBrushless);
    private CANSparkMax shooterB = new CANSparkMax(26, MotorType.kBrushless);

      SimpleMotorFeedforward feedforwardT = new SimpleMotorFeedforward(0.85, 0.0022);
  
  SimpleMotorFeedforward feedforwardB = new SimpleMotorFeedforward(0.25, 0.0022);

    public FlywheelIONEO() {
        shooterT.getPIDController().setP(0.0001);

        shooterT.setInverted(true);

        shooterB.getPIDController().setP(0.001);

        shooterB.setInverted(true);
    }


    @Override
    public void set_B_vel(double velocity) {
        
        shooterB.getPIDController().setReference(velocity, com.revrobotics.CANSparkBase.ControlType.kVelocity, 0, feedforwardB.calculate(velocity), SparkPIDController.ArbFFUnits.kVoltage);
    }

    @Override
    public void set_T_vel(double velocity) {
        SmartDashboard.putNumber("T feedforward", feedforwardT.calculate(velocity));
        shooterT.getPIDController().setReference(velocity, com.revrobotics.CANSparkBase.ControlType.kVelocity, 0,  feedforwardT.calculate(velocity), SparkPIDController.ArbFFUnits.kVoltage); 
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
