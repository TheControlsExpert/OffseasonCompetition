package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Rollers.IntakeSubsystem;
import frc.robot.subsystems.Rollers.IntakeSubsystem.Checkpoint;

public class IntakeCommand extends Command{ 

    private IntakeSubsystem intake;


    public IntakeCommand(IntakeSubsystem intake) {
        this.intake = intake;
        addRequirements(intake);
        intake.CompletedCheckpoint = Checkpoint.INITIATED;
    }


    @Override
    public void end(boolean interrupted) {
        if (intake.CompletedCheckpoint.equals(Checkpoint.INITIATED)) {
            intake.CompletedCheckpoint = Checkpoint.IDLE;

        }
    }

    


    
}
