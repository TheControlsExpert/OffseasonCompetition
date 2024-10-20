package frc.robot;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import java.util.Queue;

import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.ToDoubleFunction;

import javax.swing.plaf.basic.BasicBorders.MarginBorder;

import com.ctre.phoenix.Util;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.kauailabs.navx.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.Vision.Limelight3G.Vision_3g_Subsystem.PoseInfo;

public class RobotState {
    private final static ReentrantLock LocalizationLock = new ReentrantLock();
    private static Pose2d currentPose = new Pose2d();
    private static Pose2d odomPose = new Pose2d();
    private static TimeInterpolatableBuffer<Pose2d> poseBuffer = TimeInterpolatableBuffer.createBuffer(Constants.LocalizationConstants.BufferLifespan);
    private static double gyrovel = 0;
    //max size 500
    private static ArrayList<varianceRecord> globalVarianceList = new ArrayList<>();

    public static record varianceRecord(Matrix<N2, N2> matrix, double timestamp) {}

    private static Matrix<N2, N1> state_vector = new Matrix<>(Nat.N2(), Nat.N1());




    private static final Matrix<N2, N2> F_Matrix = new Matrix<>(Nat.N2(), Nat.N2());
    private static final Matrix<N2, N2> H_Matrix = new Matrix<>(Nat.N2(), Nat.N2());

    private static Matrix<N2, N2> P_Matrix = new Matrix<>(Nat.N2(), Nat.N2());
    private  static Matrix<N2, N2> Q_Matrix = new Matrix<>(Nat.N2(), Nat.N2());
    private static Matrix<N2, N2> R_Matrix = new Matrix<>(Nat.N2(), Nat.N2());


    public  RobotState() {
        F_Matrix.set(0, 0, 1);
        F_Matrix.set(1, 1, 1);

        H_Matrix.set(0, 0, 1);
        H_Matrix.set(1, 1, 1);

        

        P_Matrix.set(0, 0, Constants.LocalizationConstants.InitialPoseCovariancex);
        P_Matrix.set(1, 1, Constants.LocalizationConstants.InitialPoseCovariancey);
    }
    

    public static void publishOdometry(SwerveModulePosition[] positions, double yaw, double timestamp, double yawvel) {
        gyrovel = yawvel;

        LocalizationLock.lock();
        Twist2d twister = Constants.SwerveConstants.swerveKinematics.toTwist2d(positions);
        Twist2d newTwist = new Twist2d(twister.dx, twister.dy, yaw - currentPose.getRotation().getRadians());
        //prep for vision update
        odomPose.exp(newTwist);
        poseBuffer.addSample(timestamp, odomPose);
        Pose2d newpose = currentPose;
        //calculating deltas for x,y
        newpose.exp(newTwist);
        Transform2d diff = newpose.minus(currentPose);
        //calculating avg. residual
        double avg_residual_translation = 0;
        SwerveModuleState[] predicted = Constants.SwerveConstants.swerveKinematics.toSwerveModuleStates(new ChassisSpeeds(newTwist.dx, newTwist.dy, newTwist.dtheta));
        for (int i = 0; i < 4; i++) {
            double residual = Math.pow(positions[i].distanceMeters - predicted[i].speedMetersPerSecond, 2);
            avg_residual_translation += residual;
        }

        avg_residual_translation = avg_residual_translation / 4;

        //set Noise matrix for Prediction step

        Q_Matrix.set(0,0, diff.getX() * Constants.LocalizationConstants.DistanceMultiplierOdometry * (1+Constants.ResidualMultiplierOdometry * avg_residual_translation));
        Q_Matrix.set(1,1, diff.getY() * Constants.LocalizationConstants.DistanceMultiplierOdometry * (1+Constants.ResidualMultiplierOdometry * avg_residual_translation));

        //Update Noise matrix of State
        P_Matrix = F_Matrix.times(P_Matrix).times(F_Matrix.transpose()).plus(Q_Matrix);
        //Apply priori update
        state_vector.set(0, 0, currentPose.getX());
        state_vector.set(1, 0, currentPose.getY());
        globalVarianceList.add(new varianceRecord(P_Matrix, timestamp));

        LocalizationLock.unlock();

    }

    public static void setPose(Pose2d pose) {
        LocalizationLock.lock();

        currentPose = pose;

        LocalizationLock.unlock();
    }


    public static void publishVision(PoseInfo poseinfo) {

        if (!poseinfo.ignoreUpdate()) {
        LocalizationLock.lock();
        Matrix<N2, N1> measurement = new Matrix<>(Nat.N2(), Nat.N1());
        measurement.set(0, 0, poseinfo.pose().getX());
        measurement.set(1,0, poseinfo.pose().getY());

        var odomTimestamp = poseBuffer.getSample(poseinfo.timestamp());

        if (odomTimestamp.isEmpty()) {
            return;
        }

        Transform2d transform_currenttosample = odomPose.minus(odomTimestamp.get());
        Pose2d estimatedpose = currentPose.transformBy(transform_currenttosample);
        Matrix<N2,N1> estimatedposevector = VecBuilder.fill(estimatedpose.getX(), estimatedpose.getY());


        //calculate residual

        Matrix<N2,N1> residual = VecBuilder.fill(poseinfo.pose().getX() - estimatedposevector.get(0,0), poseinfo.pose().getY() - estimatedposevector.get(1,0));

        //set variance matrices

        R_Matrix.set(0, 0, Constants.LocalizationConstants.VisionMultiplierX * poseinfo.avgDistance());
        R_Matrix.set(1, 1, Constants.LocalizationConstants.VisionMultiplierY * poseinfo.avgDistance());

        //finding p matrix at time of measurement
        Matrix<N2, N2> P_Matrix_new = new Matrix<>(Nat.N2(), Nat.N2());
        int index = 0;

        for (var record : globalVarianceList) {
            if (record.timestamp() > poseinfo.timestamp()) {
                break;
            }
            P_Matrix = record.matrix();
            index = globalVarianceList.indexOf(record);
        }

        //delete all previous variance records
        globalVarianceList.subList(0, index).clear();
        

        
        //Kalman Gain
        Matrix<N2, N2> kMatrix = P_Matrix_new.times(H_Matrix.transpose()).times(H_Matrix.times(P_Matrix_new).times(H_Matrix.transpose()).plus(R_Matrix).inv());

        //choose point in residual vector


        state_vector = state_vector.plus(kMatrix.times(residual));
        currentPose = new Pose2d(state_vector.get(0, 0), state_vector.get(1, 0), currentPose.getRotation());
        


        //update covariance matrix

        Matrix<N2, N2> P_Matrix_new_new = (new Matrix<>(Nat.N2(), Nat.N2()).minus(kMatrix.times(H_Matrix))).times(P_Matrix_new);
        P_Matrix.plus(P_Matrix_new_new.minus(P_Matrix_new));


        LocalizationLock.unlock();

        }
    }



    public static Pose2d getEstimatedPosition() {
        return currentPose;
    }

    public static double getAngularVelocityFieldRel() {
        return gyrovel;
    }

    public static int getMiddleTag() {
        if (DriverStation.getAlliance().get().equals(DriverStation.Alliance.Blue)) {
            return 7;
        }

        else {
            return 4;
        }
    }


    // public static double getAligningPivotPosition() {
    //     // if (DriverStation.getAlliance().get().equals(DriverStation.Alliance.Blue)) {
    //     //     Translation2d translation = new Translation2d(, gyrovel)
    //     // }
        
    // }




}

