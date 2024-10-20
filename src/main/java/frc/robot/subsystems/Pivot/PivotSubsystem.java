package frc.robot.subsystems.Pivot;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.Pivot.PivotIO.PivotIOInputs;
import frc.robot.subsystems.Rollers.IntakeSubsystem;
import frc.robot.subsystems.Rollers.IntakeSubsystem.Checkpoint;

public class PivotSubsystem extends SubsystemBase {


    public static enum DesiredStates {
        AMP,
        SUBWOOFER,
        PODIUM,
        PASSING,
        ALIGNING,
        TRAVELLING,
        HOME
    }

   
    private DesiredStates previousDesiredState = DesiredStates.HOME;    
    private DesiredStates PivotDesiredState = DesiredStates.HOME;

    @AutoLogOutput
    private double currentSetpoint = 0.0;


    @AutoLogOutput
    private boolean atPosition = true;
    private PivotIONEO pivotIO;
    private PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();
   // private PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();
   private IntakeSubsystem intake;

   public PivotSubsystem(PivotIONEO pivotIO, IntakeSubsystem intake) {
       this.pivotIO = pivotIO;
       this.intake = intake;
   }


   @Override
   public void periodic() {
         pivotIO.updateInputs(inputs);         

         if (previousDesiredState != PivotDesiredState) {
             Logger.recordOutput("PivotDesiredState", PivotDesiredState.toString());
              previousDesiredState = PivotDesiredState;

         }

         double newsetpoint;

         if (PivotDesiredState.equals(DesiredStates.HOME)) {
            if (intake.CompletedCheckpoint.equals(Checkpoint.IDLE)) {
                newsetpoint = Constants.PivotConstants.TRAVELLING_POSITION;
            }

            else {
            newsetpoint = Constants.PivotConstants.HOME_POSITION;
            }
         }

         else if (PivotDesiredState.equals(DesiredStates.AMP)) {
            newsetpoint = Constants.PivotConstants.AMP_POSITION;
         }

         else if (PivotDesiredState.equals(DesiredStates.PODIUM)) {
            newsetpoint = Constants.PivotConstants.PODIUM_POSITION;
         }

         else if (PivotDesiredState.equals(DesiredStates.SUBWOOFER)) {
            newsetpoint = Constants.PivotConstants.SUBWOOFER_POSITION;
         }

         else if (PivotDesiredState.equals(DesiredStates.TRAVELLING)) {
            newsetpoint = Constants.PivotConstants.TRAVELLING_POSITION;
         }





        //  else if (PivotDesiredState.equals(DesiredStates.ALIGNING)) {
        //     newsetpoint = RobotState.getAligningPivotPosition();
        //     if (newsetpoint > Constants.PivotConstants.MAX_EXTENSION) {
        //         newsetpoint = Constants.PivotConstants.MAX_EXTENSION - 1;
        //     }

        //     else if (newsetpoint < Constants.PivotConstants.MIN_EXTENSION) {
        //         newsetpoint = Constants.PivotConstants.MIN_EXTENSION + 1;
        //     }
        //   }




          else if (PivotDesiredState.equals(DesiredStates.PASSING)) {
            newsetpoint = Constants.PivotConstants.PASSING_POSITION;
            if (newsetpoint > Constants.PivotConstants.MAX_EXTENSION) {
                newsetpoint = Constants.PivotConstants.MAX_EXTENSION - 1;
            }

            else if (newsetpoint < Constants.PivotConstants.MIN_EXTENSION) {
                newsetpoint = Constants.PivotConstants.MIN_EXTENSION + 1;
            }
          }

          else {
            newsetpoint = currentSetpoint;
          }


         if (currentSetpoint != newsetpoint) {
             currentSetpoint = newsetpoint;
             pivotIO.setSetpointInRotations(currentSetpoint);
         }


         if (Math.abs(currentSetpoint - inputs.encoderpos) < PivotConstants.kTolerance) {
             atPosition = true;
         }

         else {
             atPosition = false;
         }
   }



public void setDesiredState(DesiredStates state) {
       PivotDesiredState = state;
   }
    
}
