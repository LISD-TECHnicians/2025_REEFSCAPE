package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.EndEffectorSubsystem;

public class EndEffectorCommand extends Command{
    
    private final EndEffectorSubsystem m_EndEffectorSubsystem;
    private final double setSpeed;
    private final boolean limitData;

    public EndEffectorCommand(EndEffectorSubsystem subsystem, double speed, boolean data)
    {
        this.m_EndEffectorSubsystem  = subsystem;
        this.setSpeed = speed;
        this.limitData = data;
        addRequirements(m_EndEffectorSubsystem);


    }
    

  @Override
  public void initialize() {}


  @Override
  public void execute() {
    m_EndEffectorSubsystem.setEffectorSpeed(setSpeed);
  }

  @Override
  public void end(boolean interrupted) {
    m_EndEffectorSubsystem.stop();
  }


  @Override
  public boolean isFinished() {
    if (limitData == true){
        return !m_EndEffectorSubsystem.getLimitState();
    }
    return false;
  }
}
