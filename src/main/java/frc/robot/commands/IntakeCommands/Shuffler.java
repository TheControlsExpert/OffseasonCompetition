package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Rollers.IntakeSubsystem;
import frc.robot.subsystems.Rollers.IntakeSubsystem.Checkpoint;

public class Shuffler extends Command {

    private IntakeSubsystem intake;

    double initO;
    double initI;
    int counter = 0;
 
    boolean endcondition;


    public Shuffler(IntakeSubsystem intake) {
        addRequirements(intake);
        this.intake = intake;
    }

    @Override
    public void initialize() {
        endcondition = false;
        initO = Timer.getFPGATimestamp();
        intake.CompletedCheckpoint = Checkpoint.OUTTAKING;
    }

    


    

    @Override
    public void execute() {
        



         if (Timer.getFPGATimestamp() - initO > 0.1) {
            initI = Timer.getFPGATimestamp();
            intake.CompletedCheckpoint = Checkpoint.DETECTED;   
            
            if (intake.CompletedCheckpoint.equals(Checkpoint.IDLE)) {
                endcondition = true;

            }
            }


        } 


        @Override
        public boolean isFinished() {
            return endcondition;
        }
        
    }


    




    

