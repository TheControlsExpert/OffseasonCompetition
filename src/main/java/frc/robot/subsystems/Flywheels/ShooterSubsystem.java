package frc.robot.subsystems.Flywheels;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.PivotConstants;

public class ShooterSubsystem extends SubsystemBase {
    public static enum ShooterDesiredState {
        ALIGNING,
        AMP,
        PODIUM,
        SUBWOOFER,
        PASSING,
        IDLE
    }
    
    private ShooterDesiredState ShooterdesiredState = ShooterDesiredState.IDLE;
    private ShooterDesiredState previousdesiredState = ShooterDesiredState.IDLE;
    @AutoLogOutput
    private double currentSetpointB = 0;
    @AutoLogOutput
    private double currentSetpointT = 0;
    @AutoLogOutput
    private boolean atPosition = true;
    private FlywheelIONEO shooterIO;
    private FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();


    public ShooterSubsystem(FlywheelIONEO shooterIO) {
        this.shooterIO = shooterIO;
    }

    @Override
    public void periodic() {
        shooterIO.updateInputs(inputs);

         if (previousdesiredState != ShooterdesiredState) {
             Logger.recordOutput("ShooterDesiredState", ShooterdesiredState.toString());
              previousdesiredState = ShooterdesiredState;
         }



         double newsetpointB = 0;
         double newsetpointT = 0;

            if (ShooterdesiredState.equals(ShooterDesiredState.IDLE)) {
                newsetpointB = Constants.ShooterConstants.IDLE_POSITION;
                newsetpointT = Constants.ShooterConstants.IDLE_POSITION;

            }
    
            else if (ShooterdesiredState.equals(ShooterDesiredState.AMP)) {
                newsetpointB = 1200;
                newsetpointT = 300;

            }
    
            else if (ShooterdesiredState.equals(ShooterDesiredState.PODIUM)) {
                newsetpointT = 4000;
                newsetpointB = 4000;
            }
    
            else if (ShooterdesiredState.equals(ShooterDesiredState.SUBWOOFER)) {
                newsetpointB = Constants.ShooterConstants.SUBWOOFER_POSITION;
                newsetpointT = Constants.ShooterConstants.SUBWOOFER_POSITION;
            }
    
            else if (ShooterdesiredState.equals(ShooterDesiredState.PASSING)) {
                newsetpointB = 2400;
                newsetpointT = 2400;
            }
    
            // else if (ShooterdesiredState.equals(ShooterDesiredState.ALIGNING)) {
            //     newsetpointT = RobotState.getAligningShooterVelocities();
            //     newsetpointB = RobotState.getAligningShooterVelocities();

            // }

            if (newsetpointB != currentSetpointB | newsetpointT != currentSetpointT) {
                shooterIO.set_B_vel(newsetpointB);
                shooterIO.set_T_vel(newsetpointT);
                
                currentSetpointB = newsetpointB;
                currentSetpointT = newsetpointT;
        
            }


              if (Math.abs(currentSetpointT - inputs.Tvelocity) < PivotConstants.kTolerance && 
                  Math.abs(currentSetpointB - inputs.Bvelocity) < PivotConstants.kTolerance) {
                  atPosition = true;
              }
    
              else {
                  atPosition = false;
              }
             
       SmartDashboard.putNumber("shooter vel B", inputs.Bvelocity);
       SmartDashboard.putNumber("shooter setpoint T", newsetpointT);
       SmartDashboard.putNumber("shooter vel T", inputs.Tvelocity);   
         SmartDashboard.putNumber("shooter setpoint B", newsetpointB);

    }

    public void setDesiredState(ShooterDesiredState state) {
        ShooterdesiredState = state;
    }


    
    
}
