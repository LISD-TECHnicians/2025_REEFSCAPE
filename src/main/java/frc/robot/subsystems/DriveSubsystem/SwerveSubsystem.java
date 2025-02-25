package frc.robot.subsystems.DriveSubsystem;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants.PigeonConstants;
import frc.robot.Constants.RobotConstants.SwerveDriveConstants;
import frc.robot.Constants.PathPlannerConstants;
import com.ctre.phoenix6.configs.MountPoseConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveSubsystem extends SubsystemBase {
  // Declare all Swerve Modules
  private final SwerveModule frontLeftSwerve = new SwerveModule(    
    SwerveDriveConstants.kFrontLeftDriveMotorID, 
    SwerveDriveConstants.kFrontLeftRotationMotorID, 
    SwerveDriveConstants.kFrontLeftRotationEncoderID, 
    SwerveDriveConstants.kFrontLeftAngleCorrection,
    SwerveDriveConstants.kFrontLeftDriveMotorInversion,
    SwerveDriveConstants.kFrontLeftRoatationMotorInversion,
    SwerveDriveConstants.kFrontLeftRotationEncoderInversion);

  private final SwerveModule frontRightSwerve = new SwerveModule(
    SwerveDriveConstants.kFrontRightDriveMotorID, 
    SwerveDriveConstants.kFrontRightRotationMotorID, 
    SwerveDriveConstants.kFrontRightRotationEncoderID, 
    SwerveDriveConstants.kFrontRightAngleCorrection,
    SwerveDriveConstants.kFrontRightDriveMotorInversion,
    SwerveDriveConstants.kFrontRightRotationMotorInversion,
    SwerveDriveConstants.kFrontRightRotationEncoderInversion);

  private final SwerveModule rearRightSwerve = new SwerveModule(
    SwerveDriveConstants.kRearRightDriveMotorID, 
    SwerveDriveConstants.kRearRightRotationMotorID, 
    SwerveDriveConstants.kRearRightRotationEncoderID, 
    SwerveDriveConstants.kRearRightAngleCorrection,
    SwerveDriveConstants.kRearRightDriveMotorInversion,
    SwerveDriveConstants.kRearRightRotationMotorInversion,
    SwerveDriveConstants.kRearRightRotationEncoderInversion);

  public final SwerveModule rearLeftSwerve = new SwerveModule(    
    SwerveDriveConstants.kRearLeftDriveMotorID, 
    SwerveDriveConstants.kRearLeftRotationMotorID, 
    SwerveDriveConstants.kRearLeftRotationEncoderID, 
    SwerveDriveConstants.kRearLeftAngleCorrection,
    SwerveDriveConstants.kRearLeftDriveMotorInversion,
    SwerveDriveConstants.kRearLeftRotationMotorInversion,
    SwerveDriveConstants.kRearLeftRotationEncoderInversion);

  // Declare Swerve Kinematics using Swerve Module locations
  private final SwerveDriveKinematics swerve = new SwerveDriveKinematics( 
    SwerveDriveConstants.kFrontLeftModuleTranslation, 
    SwerveDriveConstants.kFrontRightModuleTranslation, 
    SwerveDriveConstants.kRearRightModuleTranslation, 
    SwerveDriveConstants.kRearLeftModuleTranslation);

  private ChassisSpeeds swerveSpeeds = new ChassisSpeeds(); // Declare Chassis Speed for use in methods
  
  private final PIDController swerveRotation = new PIDController(SwerveDriveConstants.kChassisRotation_P, SwerveDriveConstants.kChassisRotation_I, SwerveDriveConstants.kChassisRotation_D);
  private ChassisSpeeds rotationChassisSpeeds = new ChassisSpeeds();

  //  Declare Swerve Module Positions for SWerve Odometry
  private SwerveModulePosition frontLeftPosition = new SwerveModulePosition();
  private SwerveModulePosition frontRightPosition = new SwerveModulePosition();
  private SwerveModulePosition rearRightPosition = new SwerveModulePosition();
  private SwerveModulePosition rearLeftPosition = new SwerveModulePosition();

  // Inverting order might give ododmetry correct axis orientation. Needs tested
  private SwerveModulePosition[] swervePositions = {frontLeftPosition, frontRightPosition, rearRightPosition, rearLeftPosition};

  // Declare Swerve Odometry
  private final Pose2d initialPose = new Pose2d(0, 0, Rotation2d.fromRadians(0)); 

  private final SwerveDrivePoseEstimator swervePoseEstimator = new SwerveDrivePoseEstimator(swerve, Rotation2d.fromRadians(0.0), 
      swervePositions, initialPose);

  private final Pigeon2 pigeon = new Pigeon2(PigeonConstants.kPigeonID, "canivore"); // Declare IMU
  private final MountPoseConfigs mountPoseConfigs = new MountPoseConfigs().withMountPoseYaw(-90.0);

  private final Field2d fieldLayout = new Field2d();

  public SwerveSubsystem() {
    RobotConfig config;

    try{
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
      config = new RobotConfig(50, 6.883, null, null); //placeholder
    }

    AutoBuilder.configure(
      this::getPose, 
      this::setPose, 
      this::getChassisSpeeds, 
      this::setChassisSpeeds, 
      new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
        new PIDConstants(PathPlannerConstants.kPathTranslational_P, PathPlannerConstants.kPathTranslational_I, PathPlannerConstants.kPathTranslational_D), // Translation PID constants
        new PIDConstants(PathPlannerConstants.kPathRotational_P, PathPlannerConstants.kPathRotational_I,PathPlannerConstants.kPathRotational_D) // Rotation PID constants
      ),

      config,
      () -> {
          // Boolean supplier that controls when the path will be mirrored for the red alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
      },
      this
    );

    pigeon.getConfigurator().apply(new Pigeon2Configuration());
    pigeon.getConfigurator().apply(mountPoseConfigs);
  }

  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
    // List of Swerve States from desired Swerve Speeds
    SwerveModuleState[] swerveModuleStates = swerve.toSwerveModuleStates(chassisSpeeds);  

    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveDriveConstants.kMaxDriveSpeed); // Keeps motor speeds in limits

    swerveSpeeds = swerve.toChassisSpeeds(swerveModuleStates);

    frontLeftSwerve.setSwerveState(swerveModuleStates[0]); 
    frontRightSwerve.setSwerveState(swerveModuleStates[1]);
    rearRightSwerve.setSwerveState(swerveModuleStates[2]);
    rearLeftSwerve.setSwerveState(swerveModuleStates[3]);
  }

  public ChassisSpeeds getChassisSpeeds() {
    return swerveSpeeds;
  }

  public void setPose(Pose2d pose) {
    swervePoseEstimator.resetPosition(Rotation2d.fromRadians(getYaw()), swervePositions, pose);
  }
  
  public Pose2d getPose() {
    return swervePoseEstimator.getEstimatedPosition();
  }

  public void visionPoseUpdate(Pose2d visionPose, double timeStamp) {
    swervePoseEstimator.addVisionMeasurement(visionPose, timeStamp);
  }

  public double getYaw() {
    return Units.degreesToRadians(pigeon.getYaw().getValueAsDouble()); // Negative makes clockwise positive
  }

  public double getPitch() {
    return Units.degreesToRadians(pigeon.getPitch().getValueAsDouble());
  }

  public double getRoll() {
    return Units.degreesToRadians(pigeon.getRoll().getValueAsDouble());
  }

  public void setDriveBrake() {
    frontLeftSwerve.setDriveBrake(); 
    frontRightSwerve.setDriveBrake(); 
    rearRightSwerve.setDriveBrake(); 
    rearLeftSwerve.setDriveBrake();
  }

  public void setDriveCoast() {
    frontLeftSwerve.setDriveCoast(); 
    frontRightSwerve.setDriveCoast(); 
    rearRightSwerve.setDriveCoast(); 
    rearLeftSwerve.setDriveCoast();
  }

  public void setSwerveRotation(double measurement, double setpoint) {
    rotationChassisSpeeds.omegaRadiansPerSecond = swerveRotation.calculate(measurement, setpoint);
    
    setChassisSpeeds(rotationChassisSpeeds);
  }

  public void stop()
  {
    frontLeftSwerve.stop();
    frontRightSwerve.stop();
    rearRightSwerve.stop();
    rearLeftSwerve.stop();
  }

  public boolean getRotationReadiness(double measurement, double setpoint) {
    return Math.abs(measurement - setpoint) < SwerveDriveConstants.kSwerveRotationVariablility;
  }

  @Override
  public void periodic() {
    frontLeftPosition.distanceMeters = frontLeftSwerve.getDrivePosition();
    frontLeftPosition.angle = frontLeftSwerve.getSwerveState().angle;

    frontRightPosition.distanceMeters = frontRightSwerve.getDrivePosition();
    frontRightPosition.angle = frontRightSwerve.getSwerveState().angle;

    rearRightPosition.distanceMeters = rearRightSwerve.getDrivePosition();
    rearRightPosition.angle = rearRightSwerve.getSwerveState().angle;
    
    rearLeftPosition.distanceMeters = rearLeftSwerve.getDrivePosition();
    rearLeftPosition.angle = rearLeftSwerve.getSwerveState().angle;

    swervePoseEstimator.update(Rotation2d.fromRadians(getYaw()), swervePositions);

    fieldLayout.setRobotPose(getPose());
    SmartDashboard.putData("Field Layout", fieldLayout);
    SmartDashboard.putNumber("Match Time", (int)DriverStation.getMatchTime());
  }

  @Override
  public void simulationPeriodic() {}
}
