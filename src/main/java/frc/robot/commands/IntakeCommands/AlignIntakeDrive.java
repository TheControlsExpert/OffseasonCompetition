package frc.robot.commands.IntakeCommands;

import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Pivot.PivotSubsystem;
import frc.robot.subsystems.Rollers.IntakeSubsystem;
import frc.robot.subsystems.Rollers.IntakeSubsystem.Checkpoint;
import frc.robot.subsystems.SwerveActual.Swerve;
import frc.robot.subsystems.Vision.Limelight3.Limelight3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;


public class AlignIntakeDrive extends Command {





    private Swerve s_Swerve;    
    private Limelight3 ll3;
    private PIDController pidT;
    private PIDController pidR;
    private double initTime = 0;
    private boolean forwardCondition =false;
    private IntakeSubsystem intake;
    private Translation2d lastTrans = new Translation2d();
    double tyEnd = 0;
    private double lastRotation = 0;
    private PivotSubsystem pivot;

    
    public AlignIntakeDrive(Swerve s_Swerve, Limelight3 ll3, PIDController pidT, PIDController pidR, IntakeSubsystem intake, PivotSubsystem pivot) {

        this.intake = intake;
        this.s_Swerve = s_Swerve;
        this.pidT = pidT;
        this.pidR = pidR;
        this.ll3 = ll3;
        this.pivot = pivot;
        
        
        addRequirements(s_Swerve, intake, pivot);
        

        SmartDashboard.putBoolean("Swerve ON", true);
    }
    @Override
    public void initialize() {
    intake.CompletedCheckpoint = Checkpoint.INITIATED;
    pivot.setDesiredState(PivotSubsystem.DesiredStates.HOME);
    }
    
    @Override
    public void execute() {
            SmartDashboard.putBoolean("aligning ended", false);
      double[] data = ll3.getBestDetection();
      SmartDashboard.putNumber("ll3 data", data[1]);

      double tx = data[0] + 5;
      double ty = data[1];
      tyEnd = ty;
      double ta = data[2];
     
      double Dpid = 1+Math.abs(pidT.calculate(VisionConstants.Height_LL3 * 1/Math.tan(Units.degreesToRadians(Math.abs(ty-18)))));
     // SmartDashboard.putNumber("Distance PID", Dpid);
      SmartDashboard.putNumber("ty", ty);
      double Rpid = pidR.calculate(tx);
SmartDashboard.putNumber("ty", ty);
      
        






      
        /* Drive */
        
      
      
     
            if (ll3.getBestDetection()[3] != 0 ) {
               s_Swerve.drive( new Translation2d(Dpid * Math.cos(Units.degreesToRadians(tx)), Dpid *  Math.sin(Units.degreesToRadians(-tx))), Rpid, false, false);
              lastTrans =  new Translation2d(Dpid * Math.cos(Units.degreesToRadians(tx)), Dpid *  Math.sin(Units.degreesToRadians(-tx)) );
              lastRotation =  0;
      
    //  s_Swerve.drive( new Translation2d(0, 0), Rpid, false, false);
        }

        else {
          s_Swerve.drive(lastTrans, lastRotation, false, false);
          
        }
      }
      
        @Override
    public boolean isFinished() {
        return tyEnd < -18 | intake.CompletedCheckpoint.equals(Checkpoint.DETECTED);
         
         //(forwardCondition && Timer.getFPGATimestamp() - iniTime > 0.8);
        
    }


    @Override
    public void end(boolean interrupted) {
      SmartDashboard.putBoolean("aligning ended", true);
      
      if (interrupted) {
        intake.CompletedCheckpoint = Checkpoint.IDLE;
      }

    }
}
