package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem.SwerveSubsystem;
import frc.robot.Constants.RobotConstants.SwerveDriveConstants;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class SwerveCmd extends Command {
  private final SwerveSubsystem swerveSubsystem;

  private final DoubleSupplier xController; // m/s
  private final DoubleSupplier yController; // m/s
  private final DoubleSupplier rotationController; // rad/s

  private final BooleanSupplier robotOriented;

  private final BooleanSupplier slowSpeed;

  private ChassisSpeeds swerveSpeeds = new ChassisSpeeds(); 

  public SwerveCmd(SwerveSubsystem swerveSubsystem, DoubleSupplier xController, DoubleSupplier yController, 
      DoubleSupplier rotationController, BooleanSupplier robotOriented, BooleanSupplier slowSpeed) {
    this.swerveSubsystem = swerveSubsystem;
    this.xController = xController;
    this.yController = yController;
    this.rotationController = rotationController;

    this.robotOriented = robotOriented;

    this.slowSpeed = slowSpeed;

    addRequirements(swerveSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    double xSpeed = slowSpeed.getAsBoolean() ? xController.getAsDouble() * SwerveDriveConstants.kSlowRobotDriveCoef : xController.getAsDouble();
    double ySpeed = -1 * (slowSpeed.getAsBoolean() ? yController.getAsDouble() * SwerveDriveConstants.kSlowRobotDriveCoef : yController.getAsDouble());

    double rotationSpeed = slowSpeed.getAsBoolean() ? rotationController.getAsDouble() * SwerveDriveConstants.kSlowRobotRotationCoef : rotationController.getAsDouble();

    if (robotOriented.getAsBoolean()) {
      swerveSpeeds.vxMetersPerSecond = -xSpeed;
      swerveSpeeds.vyMetersPerSecond = -ySpeed;
      swerveSpeeds.omegaRadiansPerSecond = rotationSpeed;
    }
    else {
      swerveSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
          xSpeed, 
          ySpeed, 
          rotationSpeed, 
          Rotation2d.fromRadians(swerveSubsystem.getYaw()));
    }
    //System.out.println("Swerve Speeds: " + swerveSpeeds);
    swerveSubsystem.setChassisSpeeds(swerveSpeeds);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}

