package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.DriveSubsystem.SwerveSubsystem;;

public class UpdateVisionCommand extends Command {
  private final SwerveSubsystem m_SwerveSubsystem;
  private final LimelightSubsystem m_LimelightSubsystem;

  public UpdateVisionCommand(SwerveSubsystem swerveSubsystem, LimelightSubsystem limelightSubsystem) {
    this.m_SwerveSubsystem = swerveSubsystem;
    this.m_LimelightSubsystem = limelightSubsystem;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_SwerveSubsystem.visionPoseUpdate(m_LimelightSubsystem.getPose(), 
        m_LimelightSubsystem.getAdjustedTimestamp());
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
