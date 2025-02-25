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

    private final SparkMaxConfig m_RightShoulderConfig;
    //private final SparkMaxConfig m_LeftShoulderConfig;

    private final SparkMax m_RightShoudlerNeo;
    //private final SparkMax m_LeftShoudlerNeo; 


    private final RelativeEncoder m_ShoulderEncoder;


    public ArmSubsystem()
    {
        m_ShoulderController = new PIDController(ArmConstants.kShoulderRotation_P, 
                                                 ArmConstants.kShoulderRotation_I, 
                                                 ArmConstants.kShoulderRotation_D);
        

        m_RightShoulderConfig = new SparkMaxConfig();
        //m_LeftShoulderConfig = new SparkMaxConfig();

        m_RightShoudlerNeo = new SparkMax(ArmConstants.kRightShoulderMotorID, MotorType.kBrushless); // leader motor
        //m_LeftShoudlerNeo = new SparkMax(ArmConstants.kLeftShoulderMotorID, MotorType.kBrushless);

        m_ShoulderEncoder = m_RightShoudlerNeo.getEncoder();
        //m_ShoulderEncoder.setPositionConversionFactor();

        m_RightShoulderConfig.inverted(ArmConstants.kRightShoulderMotorInversion);
        m_RightShoudlerNeo.configure(m_RightShoulderConfig, null, null);

        /*m_LeftShoulderConfig.inverted(ArmConstants.kLeftShoulderMotorInversion);
        m_LeftShoulderConfig.follow(m_RightShoudlerNeo);
        m_LeftShoudlerNeo.configure(m_LeftShoulderConfig, null, null);*/
    }

    // Getters
    public double getShoulderPosition()
    {
        return m_ShoulderEncoder.getPosition() * ArmConstants.kShoulderGearRatio; 
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

    public void stop()
    {
        m_RightShoudlerNeo.set(0);
    }
}