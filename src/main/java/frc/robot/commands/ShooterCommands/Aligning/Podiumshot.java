

package frc.robot.commands.ShooterCommands.Aligning;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Flywheels.ShooterSubsystem;
import frc.robot.subsystems.Flywheels.ShooterSubsystem.ShooterDesiredState;
import frc.robot.subsystems.Pivot.PivotSubsystem;
import frc.robot.subsystems.Pivot.PivotSubsystem.DesiredStates;
import frc.robot.subsystems.Rollers.IntakeSubsystem;
import frc.robot.subsystems.Rollers.IntakeSubsystem.Checkpoint;
import frc.robot.subsystems.SwerveActual.Swerve;
import frc.robot.subsystems.Vision.Limelight3G.LimelightHelpers;
import frc.robot.subsystems.Vision.Limelight3G.LimelightHelpers.LimelightTarget_Fiducial;

public class Podiumshot extends Command {
    
    Swerve swervy;
    DoubleSupplier translationSup;
    DoubleSupplier strafeSup;
    
    private PIDController pidR;
    private ShooterSubsystem shooter;
    private PivotSubsystem pivot;
    private IntakeSubsystem intake;
    private CommandXboxController xbox;

    public Podiumshot(Swerve swervy, DoubleSupplier translationSup, DoubleSupplier strafeSup, PIDController pidR, ShooterSubsystem shooter, PivotSubsystem pivot, IntakeSubsystem intake, CommandXboxController xbox) {
        this.swervy = swervy;
        this.xbox = xbox;
        
        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.pidR = pidR;
        this.shooter = shooter;
        this.intake = intake;
        this.pivot = pivot;

        addRequirements(swervy, shooter, intake, pivot);
    }


    
    
    
    
    @Override
    public void initialize(){
      shooter.setDesiredState(ShooterDesiredState.PODIUM);
      pivot.setDesiredState(DesiredStates.PODIUM);
        
    }
    @Override
    public void execute() {
        /* Get Values, Deadband*/


        double translationVal = Math.signum(translationSup.getAsDouble()) * Math.pow(translationSup.getAsDouble(), 2);
        double strafeVal = Math.signum(strafeSup.getAsDouble()) * Math.pow(strafeSup.getAsDouble(), 2);
        

        translationVal = MathUtil.applyDeadband(translationVal, 0.05);
        strafeVal = MathUtil.applyDeadband(strafeVal, 0.05);


        
      
      double Rpid;

      double tx = SpeakerTX();
      SmartDashboard.putNumber("tx", tx);
      SmartDashboard.putNumber("detection?", SpeakerTX());

      

    //   if (data[1] != 0) {

      
       Rpid = -pidR.calculate(tx);
    //   }

    //   else {
    //    double tx = swervy.getSpeakerDetection();
    //    Rpid = pidR.calculate(tx);
    //   }
    
      



        /* Drive */
       
            swervy.drive(
                new Translation2d(-translationVal, -strafeVal).times(10), 
                -Rpid, 
                true, 
                true
                );


            if (xbox.rightTrigger().getAsBoolean()) {
            intake.CompletedCheckpoint = Checkpoint.EJECTED;        
        }
        }

       


       public double SpeakerTX() {
      
         NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-threegs");
         NetworkTableEntry tx = table.getEntry("tx");

            return tx.getDouble(0);
  
              // if (tags[i].fiducialID == PipelineConstants.SPEAKERRED && DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
              //     results[0] = tags[i].tx;
              //     results[1] = 1;
              //     break;
              // }
          
       }


       @Override
       public boolean isFinished() {
           return !intake.isTouchingLastSensor();
       }

       @Override
       public void end(boolean interrupted) {
        
           shooter.setDesiredState(ShooterDesiredState.IDLE);
           pivot.setDesiredState(DesiredStates.HOME);
           intake.CompletedCheckpoint = Checkpoint.IDLE;
       }
    
    }