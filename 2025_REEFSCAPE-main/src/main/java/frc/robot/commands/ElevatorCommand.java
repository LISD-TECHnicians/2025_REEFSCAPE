package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorCommand extends Command{
    
    private final ElevatorSubsystem m_ElevatorSubsystem;
    private final double toPosition;

    public ElevatorCommand(ElevatorSubsystem subsystem, double position)
    {
        this.m_ElevatorSubsystem  = subsystem;
        this.toPosition = position;
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
    return false; //m_ElevatorSubsystem.getLimitState(); // can move after hitting switch??
  }
}
