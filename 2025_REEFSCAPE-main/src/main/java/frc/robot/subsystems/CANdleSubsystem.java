package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants.CANdleConstants;

public class CANdleSubsystem extends SubsystemBase{

    public CANdleSubsystem()
    {}

    public double getCANdleState()
    {
        return 0.0; //CANdleConstants.Colors.getLEDVoltage();
    }

    public void setCANdleState()
    {

    }
}
