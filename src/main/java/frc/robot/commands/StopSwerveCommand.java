package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem.SwerveSubsystem;

public class StopSwerveCommand extends Command {
    private final SwerveSubsystem m_SwerveSubsystem;

    public StopSwerveCommand(SwerveSubsystem swerve) {
        this.m_SwerveSubsystem = swerve;
        addRequirements(m_SwerveSubsystem);
    }

@Override 
public void initialize() {}

@Override
public void execute() {
    m_SwerveSubsystem.stop();
}

@Override
public void end(boolean interrupted){}

@Override
public boolean isFinished() {
    return false;
}
}

