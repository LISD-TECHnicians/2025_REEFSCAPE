package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants.ArmConstants;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;


public class ArmSubsystem extends SubsystemBase{

    private final PIDController m_ShoulderController;
    private final PIDController m_WristController;

    private final SparkMaxConfig m_RightShoulderConfig;
    private final SparkMaxConfig m_LeftShoulderConfig;
    private final SparkMaxConfig m_WristConfig;

    private final SparkMax m_RightShoudlerNeo; // Assume follower
    private final SparkMax m_LeftShoudlerNeo; 
    private final SparkMax m_WristNeo; 

    private final RelativeEncoder m_ShoulderEncoder;
    private final RelativeEncoder m_WristEncoder;

    public ArmSubsystem()
    {
        m_ShoulderController = new PIDController(ArmConstants.kShoulderRotation_P, 
                                                 ArmConstants.kShoulderRotation_I, 
                                                 ArmConstants.kShoulderRotation_D);
        
        m_WristController = new PIDController(ArmConstants.kWristRotation_P, 
                                              ArmConstants.kWristRotation_I, 
                                              ArmConstants.kWristRotation_D);

        m_RightShoulderConfig = new SparkMaxConfig();
        m_LeftShoulderConfig = new SparkMaxConfig();
        m_WristConfig = new SparkMaxConfig();

        m_RightShoudlerNeo = new SparkMax(ArmConstants.kRightShoulderMotorID, MotorType.kBrushless); // leader motor
        m_LeftShoudlerNeo = new SparkMax(ArmConstants.kLeftShoulderMotorID, MotorType.kBrushless);
        m_WristNeo = new SparkMax(ArmConstants.kWristMotorID, MotorType.kBrushless);

        m_ShoulderEncoder = m_RightShoudlerNeo.getEncoder();
        //m_ShoulderEncoder.setPositionConversionFactor();
        m_WristEncoder = m_WristNeo.getEncoder();

        m_RightShoulderConfig.inverted(ArmConstants.kRightShoulderMotorInversion);
        m_RightShoudlerNeo.configure(m_RightShoulderConfig, null, null);

        m_LeftShoulderConfig.inverted(ArmConstants.kLeftShoulderMotorInversion);
        m_LeftShoulderConfig.follow(m_RightShoudlerNeo);
        m_LeftShoudlerNeo.configure(m_LeftShoulderConfig, null, null);

        m_WristConfig.inverted(ArmConstants.kWirstMotorInversion);
        m_WristNeo.configure(m_WristConfig, null, null);
    }

    // Getters
    public double getShoulderPosition()
    {
        return m_ShoulderEncoder.getPosition() * ArmConstants.kShoulderGearRatio; 
    }

    public double getWristPosition()
    {
        return m_WristEncoder.getPosition();
    }

    // Setters
    public void setShoulderPosition(double setPosition)
    {
        double currentShoulderPosition = getShoulderPosition();
        if (currentShoulderPosition > -0.25 && currentShoulderPosition > 0.25)
        // incorporate angle limits here
        setShoulderSpeed(m_ShoulderController.calculate(getShoulderPosition(), setPosition));

        else System.out.println("Shoulder Bound Error");
    }

    public void setShoulderSpeed(double setSpeed)
    {
        m_RightShoudlerNeo.set(setSpeed);
    }

    public void setWristPosition(double setPosition)
    {
        // incorporate angle limits here
        setWristSpeed(m_WristController.calculate(getWristPosition(), setPosition));
    }

    public void setWristSpeed(double setSpeed)
    {
        m_WristNeo.set(setSpeed);
    }

    public void stop()
    {
        m_RightShoudlerNeo.set(0);
        m_WristNeo.set(0);
    }
}
