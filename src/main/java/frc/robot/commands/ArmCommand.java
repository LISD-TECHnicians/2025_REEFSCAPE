package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class ArmCommand extends Command{

    private final ArmSubsystem m_ArmSubsystem;
    private final double shoulderToPosition;

    public ArmCommand(ArmSubsystem subsystem, 
                        double shoulderPosition)
    {
        this.m_ArmSubsystem = subsystem;
        this.shoulderToPosition = shoulderPosition;
        addRequirements(m_ArmSubsystem);
    }
    
    @Override
    public void initialize() {}
  
    @Override
    public void execute() {
      m_ArmSubsystem.setShoulderPosition(shoulderToPosition);
    }
  
    @Override
    public void end(boolean interrupted) {
      m_ArmSubsystem.stop(); 
    }
  
    @Override
    public boolean isFinished() {
      return false; 
    }
}
