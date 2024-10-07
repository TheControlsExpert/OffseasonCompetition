package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Rollers.IntakeSubsystem;
import frc.robot.subsystems.Rollers.IntakeSubsystem.Checkpoint;

public class OuttakeCommand extends Command{ 

    private IntakeSubsystem intake;


    public OuttakeCommand(IntakeSubsystem intake) {
        this.intake = intake;
        addRequirements(intake);
        intake.CompletedCheckpoint = Checkpoint.OUTTAKING;
    }


    @Override
    public void end(boolean interrupted) {
        
        intake.CompletedCheckpoint = Checkpoint.IDLE;

        
    }

    


    
}
