package frc.robot.subsystems;

import frc.robot.Constants.RobotConstants.ElevatorConstants;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;

public class ElevatorSubsystem extends SubsystemBase {
    
    private final SparkMax m_NeoLeader;
    private final SparkMax m_NeoFollower;
    private final RelativeEncoder m_LeadEncoder;
    private final DigitalInput m_UpperLimitSwitch;
    private final PIDController m_PidController;

    private double currentElevatorPosition;

    private final SparkMaxConfig m_LeaderSparkConfig;
    private final SparkMaxConfig m_FollowerSparkConfig; 

    public ElevatorSubsystem() {
        
        m_NeoLeader = new SparkMax(ElevatorConstants.kElevatorLeaderMotorID, MotorType.kBrushless);
        m_NeoFollower = new SparkMax(ElevatorConstants.kElevatorFollowerMotorID, MotorType.kBrushless);
        m_LeadEncoder = m_NeoLeader.getEncoder();
        m_UpperLimitSwitch = new DigitalInput(ElevatorConstants.kElevatorLimitPWM); 
      
        m_PidController = new PIDController(ElevatorConstants.kElevator_P, 
                                            ElevatorConstants.kElevator_I, 
                                            ElevatorConstants.kElevator_D);

        m_LeaderSparkConfig = new SparkMaxConfig();
        m_FollowerSparkConfig = new SparkMaxConfig();

        m_LeaderSparkConfig.inverted(false);
        m_NeoLeader.configure(m_LeaderSparkConfig, null, null);
       
        m_FollowerSparkConfig.inverted(true);
        m_FollowerSparkConfig.follow(m_NeoLeader);
        m_NeoFollower.configure(m_FollowerSparkConfig, null, null);
        
        // New API has no correction factor? --> Need a manually created method to calc wheel speeds. 
        m_LeadEncoder.setPosition(ElevatorConstants.kElevatorOffset); 
    }
    
    // getters
    public double getElevatorPosition()
    {
        return m_LeadEncoder.getPosition(); // corrected for meters
    }

    public double getSpeed()
    {
        return m_NeoLeader.get();
    }

    public Boolean getLimitState()
    {
        return m_UpperLimitSwitch.get();
    }


    // Setters
    public void setSpeed(double speed)
    {
        m_NeoLeader.set(speed);
    }

    public void setElevatorPosition(double targetPosition)
    {
        currentElevatorPosition = getElevatorPosition();
        setSpeed(m_PidController.calculate(currentElevatorPosition, targetPosition)); // Integrate the PID controller here 
    }

    public void stop()
    {
         m_NeoLeader.set(0);
    }

    public void resetEncoder()
    {
        m_LeadEncoder.setPosition(0);
    }
}
