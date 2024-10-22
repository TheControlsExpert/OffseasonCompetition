package frc.robot.commands.ShooterCommands.Aligning;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Pivot.PivotSubsystem;
import frc.robot.subsystems.Pivot.PivotSubsystem.DesiredStates;

public class ZeroPivot extends Command {

    private PivotSubsystem pivot;
    private CommandXboxController copilot;

    public ZeroPivot(PivotSubsystem pivot, CommandXboxController copilot) {
        this.pivot= pivot;
        this.copilot = copilot;
    }



    @Override
    public void initialize() {
        pivot.setDesiredState(DesiredStates.MANUAL);
    }

    @Override
    public void execute() {
        if (copilot.leftBumper().getAsBoolean()) {
            pivot.set(0.3);
        }
    }



    @Override
    public boolean isFinished() {
        return copilot.rightBumper().getAsBoolean();

    }

    @Override
    public void end(boolean interrupted) {
        pivot.resetEncoder();
        pivot.setDesiredState(DesiredStates.TRAVELLING);
    }
    
}
