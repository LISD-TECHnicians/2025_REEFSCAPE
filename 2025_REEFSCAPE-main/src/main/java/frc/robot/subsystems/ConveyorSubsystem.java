package frc.robot.subsystems;
import frc.robot.Constants.RobotConstants.ConveyorConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;



public class ConveyorSubsystem extends SubsystemBase {

    private final SparkMax m_ConveyorDriveMotor;
    private final DigitalInput m_ConveyorCoralSensor;
    private final SparkMaxConfig m_DefaultSparkConfig;

    public ConveyorSubsystem() {
        m_ConveyorDriveMotor = new SparkMax(ConveyorConstants.kConveyorDriveMotorID, MotorType.kBrushless);
        m_ConveyorCoralSensor = new DigitalInput(ConveyorConstants.kConveyorLimitPWM);
        m_DefaultSparkConfig = new SparkMaxConfig (); // configuration object 

        m_DefaultSparkConfig.inverted(ConveyorConstants.kConveyorDriveMotorInversion);
        m_ConveyorDriveMotor.configure(m_DefaultSparkConfig, null, null);
    }

        // getters 
        public boolean getLimitState()
        {
            return m_ConveyorCoralSensor.get();
        }


        // setters 
        public void setSpeed(double setSpeed)
        {
            m_ConveyorDriveMotor.set(setSpeed);
        }

        public void stop()
        {
            m_ConveyorDriveMotor.set(0);
        }
}
