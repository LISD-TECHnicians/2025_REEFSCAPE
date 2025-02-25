package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants.EndEffectorConstants;
import edu.wpi.first.wpilibj.DigitalInput;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import com.revrobotics.spark.config.SparkMaxConfig;

public class EndEffectorSubsystem extends SubsystemBase{
    
    private final SparkMax m_LeaderEffectorMotor;
    //private final SparkMax m_FollowerEffectorMotor;
   
    //private final SparkMax m_LeaderWristMotor;
    //private final SparkMax m_FollowerWristMotor;

    private final SparkMaxConfig m_LeaderEffectorConfig;
    //private final SparkMaxConfig m_FollowerEffectorConfig;
    
    //private final SparkMaxConfig m_LeaderWristConfig;
    //private final SparkMaxConfig m_FollowerWristConfig;

    private final DigitalInput m_EffectorCoralSensor;

    //private final RelativeEncoder m_WristEncoder;

    private final PIDController m_PidController; 

    
    public EndEffectorSubsystem()
    {
        m_LeaderEffectorMotor = new SparkMax(EndEffectorConstants.kLeaderEffectorMotorID, MotorType.kBrushless);
        //m_FollowerEffectorMotor = new SparkMax(EndEffectorConstants.kFollowerEffectorMotorID, MotorType.kBrushless);

        //m_LeaderWristMotor = new SparkMax(EndEffectorConstants.kLeaderEffectorWristMotorID,  MotorType.kBrushless);
        //m_FollowerWristMotor = new SparkMax(EndEffectorConstants.kFollowerEffectorWristMotorID,  MotorType.kBrushless);

        m_LeaderEffectorConfig = new SparkMaxConfig();
        //m_FollowerEffectorConfig = new SparkMaxConfig();

        //m_LeaderWristConfig = new SparkMaxConfig();
        //m_FollowerWristConfig = new SparkMaxConfig();

        m_EffectorCoralSensor = new DigitalInput(EndEffectorConstants.kEffectorLimitDI);

        //m_WristEncoder = m_LeaderWristMotor.getEncoder();

        m_PidController = new PIDController(EndEffectorConstants.kEffectorWrist_P,
                                            EndEffectorConstants.kEffectorWrist_I,
                                            EndEffectorConstants.kEffectorWrist_D);


        m_LeaderEffectorConfig.inverted(false);
        m_LeaderEffectorConfig.smartCurrentLimit(40, 40);
        m_LeaderEffectorMotor.configure(m_LeaderEffectorConfig, null, null);

       /*  m_FollowerEffectorConfig.inverted(true);
        m_FollowerEffectorConfig.follow(m_LeaderEffectorMotor);
        m_FollowerEffectorMotor.configure(m_FollowerEffectorConfig, null, null); */

       // m_LeaderWristConfig.inverted(false);
        //m_LeaderWristMotor.configure(m_LeaderWristConfig, null, null);

       /*  m_FollowerWristConfig.inverted(true);
        m_FollowerWristConfig.follow(m_LeaderWristMotor);
        m_FollowerEffectorMotor.configure(m_FollowerWristConfig, null, null); */
       
        // New API has no correction factor? --> Need a manually created method to calc wheel speeds.
       
        //resetEncoders(); // zero wrist encoder 
    }
    

    // getters

    /*public double getWristAngle()
    {
        return  m_WristEncoder.getPosition();
    }*/

    public boolean getLimitState()
    {
        return m_EffectorCoralSensor.get();
    }


    // setters


    public void setEffectorSpeed(double setSpeed)
    {
        m_LeaderEffectorMotor.set(setSpeed);
    }

    public void setEndEffectorPosition(double position)
    {
        m_LeaderEffectorMotor.set(position);
    }

    /*public void setWristSpeed(double setSpeed)
    {
        m_LeaderWristMotor.set(setSpeed);
    }

    public void setWristAngle(double setAngle)
    {
        
    }*/

    /*public void resetEncoders()
    {
        m_WristEncoder.setPosition(0);
    }*/

    public void stop()
    {
        m_LeaderEffectorMotor.set(0);
        //m_FollowerEffectorMotor.set(0);

        //m_LeaderWristMotor.set(0);
        //m_FollowerWristMotor.set(0);
    }
}
