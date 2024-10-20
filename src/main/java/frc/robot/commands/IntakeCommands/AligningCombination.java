package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AligningCombination extends SequentialCommandGroup {
    public AligningCombination(AlignIntakeDrive aligner, DriveTime drive) {
        addCommands(aligner, drive);
    }



    
    
}
