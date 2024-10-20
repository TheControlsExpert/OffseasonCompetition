package frc.robot.subsystems.Rollers;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Rollers.RollersIO.RollersIOInputs;
import frc.robot.subsystems.Rollers.SensorsIO.SensorsIOInputs;


public class IntakeSubsystem extends SubsystemBase {



    public enum Checkpoint {
        //intaking status
        DETECTED,
        IDLE,
       

        //outtaking status
        OUTTAKING,
        MANUAL_INTAKING,   

        //No note or just initialized (we don't want intake turning on)
        //IDLE,
        //No note, but intake on
        INITIATED,

        //Shooting

        EJECTED
    }


    @AutoLogOutput(key = "IntakeSubsystem-LastCompletedCheckpoint")
    public Checkpoint CompletedCheckpoint = Checkpoint.IDLE;

    private  RollersIONEO Rollersio; 
    private final RollersIOInputsAutoLogged inputs_rollers = new RollersIOInputsAutoLogged();

    //private final SensorsIO Sensorsio = new SensorsIO();
    
    
    private SensorsIO Sensorsio;
    private final SensorsIOInputsAutoLogged inputs_sensors = new SensorsIOInputsAutoLogged();
    
    //private final SensorsIOInputsAutoLogged 
    //private final SensorsIOInputsAutoLogged inputs_sensors = new SensorsIOInputsAutoLogged();

    public IntakeSubsystem(RollersIONEO Rollersio, SensorsIO Sensorsio) {
        this.Rollersio = Rollersio;
        this.Sensorsio = Sensorsio;

        //updating here as the input default values make no sense. Ex. 0 Celsius motors?
        this.Rollersio.updateInputs(inputs_rollers);
        this.Sensorsio.updateInputs(inputs_sensors);
    }



    @Override
    public void periodic() {
        
        Rollersio.updateInputs(inputs_rollers);
        Sensorsio.updateInputs(inputs_sensors);
        SmartDashboard.putString("intake state", (CompletedCheckpoint.toString()));

       
        //state transitioning 

        // if (CompletedCheckpoint == Checkpoint.IDLE && (inputs_sensors.lastReading)) {
        //     //happens at the start if there's a note inside and skips all intake logic
        //     CompletedCheckpoint = Checkpoint.STOWED;
        // }


         if (CompletedCheckpoint.equals(Checkpoint.INITIATED) && !inputs_sensors.firstReading) {
            CompletedCheckpoint = Checkpoint.DETECTED;
 
        }

        else if (CompletedCheckpoint.equals(Checkpoint.INITIATED) && !inputs_sensors.lastReading) {
            CompletedCheckpoint = Checkpoint.IDLE;
        }

         if (CompletedCheckpoint.equals(Checkpoint.DETECTED) && !inputs_sensors.lastReading) {
            CompletedCheckpoint = Checkpoint.IDLE;
        }

         if (CompletedCheckpoint.equals(Checkpoint.EJECTED) && inputs_sensors.firstReading && inputs_sensors.lastReading) {
            CompletedCheckpoint = Checkpoint.IDLE;  
          

        }

        //state control

        if (CompletedCheckpoint.equals(Checkpoint.INITIATED)) { 
            Rollersio.setSpeedHandoff(0.3);
            Rollersio.setSpeedLeftIntake(0.65);
            Rollersio.setSpeedRightIntake(0.65);

        }

       else if (CompletedCheckpoint.equals(CompletedCheckpoint.DETECTED)) { 
        Rollersio.setSpeedHandoff(0.2);
        Rollersio.setSpeedLeftIntake(0.4);
        Rollersio.setSpeedRightIntake(0.4);

       }
        

        else if (CompletedCheckpoint.equals(Checkpoint.IDLE)) {
            Rollersio.setSpeedHandoff(0);
            Rollersio.setSpeedLeftIntake(0);
            Rollersio.setSpeedRightIntake(0);
            
        }

        else if (CompletedCheckpoint.equals(Checkpoint.EJECTED)) {
            if (!inputs_sensors.lastReading) {
                Rollersio.setSpeedHandoff(0.5);
                Rollersio.setSpeedLeftIntake(0);
                Rollersio.setSpeedRightIntake(0);
            }

            else {
                Rollersio.setSpeedHandoff(0.5);
                Rollersio.setSpeedLeftIntake(0.4);
                Rollersio.setSpeedRightIntake(0.4);
            }

        }


        else if (CompletedCheckpoint.equals(Checkpoint.MANUAL_INTAKING)) {
            Rollersio.setSpeedHandoff(0.35);
            Rollersio.setSpeedLeftIntake(0.4);
            Rollersio.setSpeedRightIntake(0.4);


        } 


        else if (CompletedCheckpoint.equals(Checkpoint.OUTTAKING)) {
            Rollersio.setSpeedHandoff(-0.6);
            Rollersio.setSpeedLeftIntake(-0.6);
            Rollersio.setSpeedRightIntake(-0.6);

        }
     
    }


    public boolean isTouchingLastSensor() {
        return !inputs_sensors.lastReading;
    }


}