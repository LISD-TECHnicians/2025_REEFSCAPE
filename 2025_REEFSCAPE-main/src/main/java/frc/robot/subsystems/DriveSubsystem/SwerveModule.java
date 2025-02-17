package frc.robot.subsystems.DriveSubsystem;

import frc.robot.Constants.RobotConstants.SwerveDriveConstants;
import edu.wpi.first.math.MathUtil;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModule {
  private final TalonFX driveMotor;
  private final SparkMax rotationMotor;

  private final CANcoder rotationEncoder;
  private final double angleOffset;

  private final boolean rotationEncoderInvert;

  private final SlewRateLimiter driveLimiter = new SlewRateLimiter(SwerveDriveConstants.kMaxDriveAcceleration);

  private final PIDController rotationPID = new PIDController(SwerveDriveConstants.kRotationMotor_P, SwerveDriveConstants.kRotationMotor_I, 
  SwerveDriveConstants.kRotationMotor_D);

  private MotorOutputConfigs driveConfig = new MotorOutputConfigs();
  private SparkMaxConfig rotationConfig = new SparkMaxConfig();

  private SwerveModuleState currentSwerveModuleState = new SwerveModuleState();

  public SwerveModule(int driveMotorID, int rotationMotorID, int rotationEncoderID, double angleOffset, 
      boolean driveMotorInvert, boolean rotationMotorInvert, boolean rotationEncoderInvert) {
    // Declare swerve module componenets
    driveMotor = new TalonFX(driveMotorID, "canivore");
    rotationMotor = new SparkMax(rotationMotorID, MotorType.kBrushless);

    rotationEncoder = new CANcoder(rotationEncoderID, "canivore");

    // Clear any left over settings from previous uses
    driveConfig = driveConfig.withNeutralMode(NeutralModeValue.Brake);
    driveConfig = driveConfig.withInverted(driveMotorInvert ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive);
    driveMotor.getConfigurator().apply(driveConfig);

    rotationConfig.smartCurrentLimit(SwerveDriveConstants.kCurrentLimitRotation);
    rotationConfig.idleMode(IdleMode.kBrake);
    rotationConfig.inverted(rotationMotorInvert);
    rotationMotor.configure(rotationConfig, null, null);

    rotationEncoder.getConfigurator().apply(new CANcoderConfiguration());

    this.rotationEncoderInvert = rotationEncoderInvert;


    this.angleOffset = angleOffset; // Offsets built in error from Absolute Encoder

    // Pevents rotation motor from rotating more than 90 deg
    rotationPID.enableContinuousInput(-Math.PI, Math.PI);
  }

  public double getDrivePosition() { // Returns meters
    return driveMotor.getRotorPosition().getValueAsDouble() * SwerveDriveConstants.kDriveMotortoMeters;
  }

  public double getDriveVelocity() { // Returns meters per second
    return driveMotor.getRotorVelocity().getValueAsDouble() * SwerveDriveConstants.kDriveMotorVelocitytoMS; 
  }

  public double getRotationPosition() { // Returns radians
    return (Units.rotationsToRadians(rotationEncoder.getAbsolutePosition().getValueAsDouble()) - angleOffset) * (rotationEncoderInvert ? -1 : 1); 
  }

  /*public void setRotationPosition(double setpoint) {
    double rotationSpeed = rotationPID.calculate(getRotationPosition(), setpoint);
    //System.out.println(setpoint);
    rotationSpeed = SwerveDriveConstants.kNominalVoltage * rotationSpeed * -SwerveDriveConstants.kRotationSpeedCoef;
    //System.out.println(rotationSpeed);
    rotationSpeed = Math.abs(rotationSpeed) > 1.0 ? rotationSpeed : 0;
    rotationMotor.setVoltage(rotationSpeed);
  }*/

  public double getRotationVelocity() { // Returns radians per second
    return Units.rotationsToRadians(rotationEncoder.getVelocity().getValueAsDouble()) * (rotationEncoderInvert ? -1 : 1);
  }

  public SwerveModuleState getSwerveState() {
    currentSwerveModuleState.speedMetersPerSecond = getDriveVelocity();
    currentSwerveModuleState.angle = Rotation2d.fromRadians(getRotationPosition());

    return currentSwerveModuleState;
  }

  public void setSwerveState(SwerveModuleState swerveModuleState) {
    swerveModuleState.optimize(getSwerveState().angle);
    //System.out.println(this + " serial swerve value: " + swerveModuleState.angle.getRadians());

    double driveSpeed = driveLimiter.calculate(swerveModuleState.speedMetersPerSecond);
    driveSpeed = SwerveDriveConstants.kNominalVoltage * driveSpeed / SwerveDriveConstants.kMaxDriveSpeed;
    driveSpeed = Math.abs(driveSpeed) > SwerveDriveConstants.kDriveVoltageDeadband ? driveSpeed : 0;

    double rotationSpeed = rotationPID.calculate(getRotationPosition(), swerveModuleState.angle.getRadians());
    //SmartDashboard.putData("rotationPID", rotationPID);
    rotationSpeed = SwerveDriveConstants.kNominalVoltage * MathUtil.clamp(rotationSpeed, -SwerveDriveConstants.kRotationSpeedCoef, 
    SwerveDriveConstants.kRotationSpeedCoef);
    rotationSpeed = Math.abs(rotationSpeed) > 1.5 ? rotationSpeed : 0;
    //SmartDashboard.putNumber("Voltage", rotationSpeed);
    driveMotor.setVoltage(driveSpeed);
    rotationMotor.setVoltage(rotationSpeed);
  }

  public void setDriveBrake() {
    driveMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  public void setDriveCoast() {
    driveMotor.setNeutralMode(NeutralModeValue.Coast);
  }
}