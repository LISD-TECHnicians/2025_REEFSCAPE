package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants.EndEffectorConstants;
import edu.wpi.first.wpilibj.DigitalInput;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

public class EndEffectorSubsystem extends SubsystemBase{
    
    private final SparkMax m_EndEffectorMotor;
    private final SparkMaxConfig m_EndEffectorConfig;
    private final DigitalInput m_EffectorCoralSensor;

    public EndEffectorSubsystem()
    {
        m_EndEffectorMotor = new SparkMax(EndEffectorConstants.kLeaderEffectorMotorID, MotorType.kBrushless);
        m_EndEffectorConfig = new SparkMaxConfig();
        m_EffectorCoralSensor = new DigitalInput(EndEffectorConstants.kEffectorLimitDI);

        m_EndEffectorConfig.inverted(false);
        m_EndEffectorConfig.smartCurrentLimit(40, 40);
        m_EndEffectorMotor.configure(m_EndEffectorConfig, null, null);
    }
    
    public boolean getLimitState()
    {
        return m_EffectorCoralSensor.get();
    }

    public void setEffectorSpeed(double setSpeed)
    {
        m_EndEffectorMotor.set(setSpeed);
    }

    public void setEndEffectorPosition(double position)
    {
        m_EndEffectorMotor.set(position);
    }

    public void stop()
    {
        m_EndEffectorMotor.set(0);
    }
}
