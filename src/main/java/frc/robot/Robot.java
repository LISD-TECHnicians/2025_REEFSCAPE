package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import edu.wpi.first.wpilibj.TimedRobot; //trail 
import java.io.OutputStream;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  private UsbCamera m_UsbCamera;
  private CvSink m_ImageSink;
  private CvSource m_OutputStream;
  

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    //m_UsbCamera = new UsbCamera("Webcam1", 1);
    CameraServer.startAutomaticCapture();
    }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    /*CameraServer.startAutomaticCapture(1);
    m_ImageSink = CameraServer.getVideo(m_UsbCamera);
    m_OutputStream = CameraServer.putVideo("VideoStream", 640, 480)*/

  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}
}
