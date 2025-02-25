package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants.LimelightConstants;
import frc.robot.LimelightHelpers;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;


public class LimelightSubsystem extends SubsystemBase {

    public LimelightSubsystem() {
        LimelightHelpers.setPipelineIndex(LimelightConstants.kOdometryLimelightName, 0);
    }  

    public Pose2d getPose() 
    {
        return LimelightHelpers.getBotPose2d_wpiBlue(LimelightConstants.kOdometryLimelightName);
    }

    public boolean getValidTag()
    {
        return LimelightHelpers.getTV(LimelightConstants.kOdometryLimelightName);
    }

    public double getOdometryLimelight_Tx()
    {
        return LimelightHelpers.getTX(LimelightConstants.kOdometryLimelightName);
    }
    
    public double getOdometryLimelight_Ty()
    {
        return LimelightHelpers.getTY(LimelightConstants.kOdometryLimelightName);
    }

    public double getOdometryLimelight_Ta()
    {
        return LimelightHelpers.getTA(LimelightConstants.kOdometryLimelightName);
    }

    public double getOdometryLimelight_LatencyPipeline()
    {
        return LimelightHelpers.getLatency_Pipeline(LimelightConstants.kOdometryLimelightName);
    }

    public double getOdometryLimelight_LatencyCapture()
    {
        return LimelightHelpers.getLatency_Capture(LimelightConstants.kOdometryLimelightName);
    }


    // Target Limelight 
    public double getTargetLimelight_Tx()
    {
        return LimelightHelpers.getTX(LimelightConstants.kTargetLimelightName);
    }
    
    public double getTargetLimelight_Ty()
    {
        return LimelightHelpers.getTY(LimelightConstants.kTargetLimelightName);
    }

    public double getTargetLimelight_Ta()
    {
        return LimelightHelpers.getTA(LimelightConstants.kTargetLimelightName);
    }

    public double getTargetLimelight_LatencyPipeline()
    {
        return LimelightHelpers.getLatency_Pipeline(LimelightConstants.kTargetLimelightName);
    }

    public double getTargetLimelight_LatencyCapture()
    {
        return LimelightHelpers.getLatency_Capture(LimelightConstants.kTargetLimelightName);
    }


    // general 
    public double getRawTimeStamp()
    {
        return Timer.getFPGATimestamp();
    }

    public double getAdjustedTimestamp()
    {
        return getRawTimeStamp() - 
        (getOdometryLimelight_LatencyCapture() + 
         getOdometryLimelight_LatencyPipeline()) / 1000;
    }
}
