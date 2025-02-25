package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorCommand extends Command{
    
    private final ElevatorSubsystem m_ElevatorSubsystem;
    private final double toPosition;
    private final boolean limitData;

    public ElevatorCommand(ElevatorSubsystem subsystem, double position, boolean data)
    {
        this.m_ElevatorSubsystem  = subsystem;
        this.toPosition = position;
        this.limitData = data;
        addRequirements(m_ElevatorSubsystem);
    }
    
  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_ElevatorSubsystem.setElevatorPosition(toPosition);
  }

  @Override
  public void end(boolean interrupted) {
    m_ElevatorSubsystem.stop();
  }

  @Override
  public boolean isFinished() {
    if (limitData == true){
      return !m_ElevatorSubsystem.getLimitState();
  }
  return false;
  }
}
