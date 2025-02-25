package frc.robot;

import java.lang.Math;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public final class Constants {

  public static class RobotConstants 
  {
    public static class ArmConstants
    {
      public static final int kRightShoulderMotorID = 23;
      public static final boolean kRightShoulderMotorInversion = false;
  
      public static final double kShoulderGearRatio = 1.0;

      // test and establish these 
      public static final double kShoulderRotation_P = 1.0; 
      public static final double kShoulderRotation_I = 0.0; 
      public static final double kShoulderRotation_D = 0.0; 

      public static final double kMinShoulderAngle = -0.25; 
      public static final double kMaxShoulderAngle = 0;
    }

    public static class CANdleConstants
    {
      public static final int CANdleID = 27; 

      public enum Colors 
      {
        RED(0.0),
        BLUE(0.0),
        GREEN(0.0);

        private final double LEDVoltage;

        Colors(double LEDVoltage)
        {
          this.LEDVoltage = LEDVoltage;
        }

        public double getLEDVoltage()
        {
          return LEDVoltage; 
        }
      }
    }

    public static class ConveyorConstants
    {
      public static final int kConveyorDriveMotorID = 18;
      public static final boolean kConveyorDriveMotorInversion = false; 
      public static final double kConveyorDriveSpeed = .20;
      public static final int kConveyorLimitPWM = 2; 
    }

    public static class ElevatorConstants
    {
      public static final int kElevatorLeaderMotorID = 16;
      public static final int kElevatorFollowerMotorID = 17;
      public static final int kElevatorLimitPWM = 1; // PWM 
      public static final double kElevatorDriveSpeed = .20;
      public static final double kElevatorMotorRotationstoMeter = 1.0 / 100.0; // dummy values
      public static final double kElevatorOffset = .25; // meters
      public static final double kElevator_P = 0.5; // default values --> change later 
      public static final double kElevator_I = 0.0; 
      public static final double kElevator_D = 0.0; 
    }

    public static class EndEffectorConstants
    {
      public static final int kLeaderEffectorMotorID = 20;
      public static final int kEffectorLimitDI = 9; 
      public static final double kEffectorMotorSpeed = .15; 
      public static final double kEffectorWrist_P = 1.0; // default values --> change later 
      public static final double kEffectorWrist_I = 0.0; 
      public static final double kEffectorWrist_D = 0.0; 

      public static final int[] kPIDTransversal = {1,1,1}; 
      public static final int[] kPIDRotational = {1,1,1};
    }

    public static class LimelightConstants 
    {
      // update this 
      public static final String kOdometryLimelightName = "limelight";
      public static final String kTargetLimelightName = "";
      public static double kMin_TA = .30;
      public static double kMax_Tx = 20;
    }

    public static class PigeonConstants
    {
      public static final int kPigeonID = 13; 
      public static final boolean kInvertPigeonReading = false; 
    }

    public static class ShooterConstants
    {
     public static final int kMainShooterMotorID = 14; 
      public static final boolean kInvertMainShooter = false;  
    }

    public static class SwerveDriveConstants
    {
      public static final int kNominalVoltage = 11;

      public static final int kFrontLeftDriveMotorID = 2;
      public static final int kFrontLeftRotationMotorID = 6;
      public static final int kFrontLeftRotationEncoderID = 10;
      public static final double kFrontLeftAngleCorrection = 2.638; //2.653
      public static final boolean kFrontLeftDriveMotorInversion = false;
      public static final boolean kFrontLeftRoatationMotorInversion = true;
      public static final boolean kFrontLeftRotationEncoderInversion = false;

      public static final int kFrontRightDriveMotorID = 3;
      public static final int kFrontRightRotationMotorID = 7;
      public static final int kFrontRightRotationEncoderID = 11;
      public static final double kFrontRightAngleCorrection = 3.005;
      public static final boolean kFrontRightDriveMotorInversion = true;
      public static final boolean kFrontRightRotationMotorInversion = true;
      public static final boolean kFrontRightRotationEncoderInversion = false;

      public static final int kRearRightDriveMotorID = 4;
      public static final int kRearRightRotationMotorID = 8;
      public static final int kRearRightRotationEncoderID = 12;
      public static final double kRearRightAngleCorrection = 0.667; // In radians
      public static final boolean kRearRightDriveMotorInversion = true;
      public static final boolean kRearRightRotationMotorInversion = true;
      public static final boolean kRearRightRotationEncoderInversion = false;

      public static final int kRearLeftDriveMotorID = 1;
      public static final int kRearLeftRotationMotorID = 5;
      public static final int kRearLeftRotationEncoderID = 9;
      public static final double kRearLeftAngleCorrection = 2.356;
      public static final boolean kRearLeftDriveMotorInversion = false;
      public static final boolean kRearLeftRotationMotorInversion = true;
      public static final boolean kRearLeftRotationEncoderInversion = false;

      // About robot center
      // X AXIS TOWARDS FRONT, Y AXIS TOWARDS LEFT
      public static final Translation2d kFrontLeftModuleTranslation = new Translation2d(0.3587, 0.2699); 
      public static final Translation2d kFrontRightModuleTranslation = new Translation2d(0.3587, -0.2699); 
      public static final Translation2d kRearRightModuleTranslation = new Translation2d(-0.3587, -0.2699); 
      public static final Translation2d kRearLeftModuleTranslation = new Translation2d(-0.3587, 0.2699); 

      public static final int kCurrentLimitDrive = 40;
      public static final int kCurrentLimitRotation = 40;

      public static final double kSwerveModuleRadius = 0.448; // m

      public static final double kServeModuleWheelDiameter = 0.102;
      public static final double kSwerveModuleWheelCircumference = kServeModuleWheelDiameter * Math.PI;

      public static final double kSwerveGearRatioDrive = 1/5.9;

      public static final double kDriveMotortoMeters = kSwerveGearRatioDrive * kSwerveModuleWheelCircumference;
      public static final double kDriveMotorVelocitytoMS = kSwerveGearRatioDrive * kSwerveModuleWheelCircumference;

      public static final double kMaxDriveSpeed = 4.97; // Max possible m/s
      public static final double kMaxDriveAcceleration = 20; // Max choosen m/s^2

      public static final double kMaxRotationSpeed = kMaxDriveSpeed / kSwerveModuleRadius;

      public static final double kRotationSpeedCoef = 1;

      public static final double MaxRotationSpeedCorrected = kMaxRotationSpeed * kRotationSpeedCoef;

      public static final double kRotationMotor_P = 0.3;
      public static final double kRotationMotor_I = 0.0; 
      public static final double kRotationMotor_D = 0.0;

      public static final double kChassisRotation_P = 0.15;
      public static final double kChassisRotation_I = 0.040;
      public static final double kChassisRotation_D = 0.0;

      public static final double kDriveVoltageDeadband = 1.5;

      public static final Pose2d InitPose = new Pose2d();

      public static final ChassisSpeeds kInverseIntakeVelocity = new ChassisSpeeds(-1.0, 0, 0);

      public static final double kSwerveIntakeOffset = 2.0;
      public static final double kSweverShooterOffset = 2.0;

      public static final double kSwerveRotationVariablility = 2.0;

      public static final double kSlowRobotDriveCoef = 0.45;
      public static final double kSlowRobotRotationCoef = 0.45;
    } 
  } // END RobotConstants 

  public static class OperatorConstants 
  {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1; 

    public static final int kDriverSlewRateLimitCoefX = 2;
    public static final int kDriverSlewRateLimitCoefY = 2;
    public static final int kDriverSlewRateLimitCoefTheta = 2;
    
    public static final int kOperatorSlewRateLimitCoefX = 2;
    public static final int kOperatorSlewRateLimitCoefY = 2;
    public static final int kOperatorSlewRateLimitCoefTheta = 2;

    public static final double kDriverDeadbandX = .05d;
    public static final double kDriverDeadbandY = .05d;
    public static final double kDriverDeadbandTheta = .05d;

    public static final double kOperatorDeadbandX = .05d;
    public static final double kOperatorDeadbandY = .05d;
    public static final double kOperatorDeadbandTheta = .05d;
  }

  public static class FieldConstants
  {
    /* Original measures.
     * L1 - .46
     * L2 - .81
     * L3 - 1.21
     * L4 - 1.83
    */
    public static final double kElevatorHome = 0.0;

    public static final double kCoralBranchHeight_L1 = .49; 
    public static final double kCoralBranchHeight_L2 = .86; 
    public static final double kCoralBranchHeight_L3 = 1.26;
    public static final double kCoralBranchHeight_L4 = 1.86;
  }

  public static class PathPlannerConstants
  {
    public static final double kPathTranslational_P = 4.5;
    public static final double kPathTranslational_I = 0; 
    public static final double kPathTranslational_D = 0;  

    public static final double kPathRotational_P = 2.5;
    public static final double kPathRotational_I = 0;
    public static final double kPathRotational_D = 0;
  }
} // END Constants 
