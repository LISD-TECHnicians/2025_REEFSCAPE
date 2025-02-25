// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.commands.ArmCommand;
import frc.robot.commands.Autos;
import frc.robot.commands.TeleopDriveCommand;
import frc.robot.commands.ElevatorCommand;
import frc.robot.commands.SwerveCmd;
import frc.robot.subsystems.DriveSubsystem.SwerveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.commands.EndEffectorCommand;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.Constants.RobotConstants.EndEffectorConstants;
import frc.robot.Constants.RobotConstants.SwerveDriveConstants;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {

  private final CommandXboxController m_DriverController =
    new CommandXboxController(OperatorConstants.kDriverControllerPort);
  
  private final CommandXboxController m_OperatorController = 
    new CommandXboxController(OperatorConstants.kOperatorControllerPort);

  private final ElevatorSubsystem m_ElevatorSubsystem = 
    new ElevatorSubsystem();

  private final EndEffectorSubsystem m_EndEffectorSubsystem = 
    new EndEffectorSubsystem();

  private final ArmSubsystem m_ArmSubsystem = 
    new ArmSubsystem();

    private final SwerveSubsystem m_SwerveSubsystem = new SwerveSubsystem();

private final SwerveCmd joystickSwerve = new SwerveCmd(
      m_SwerveSubsystem, 
      () -> Math.abs(m_DriverController.getLeftX()) >= OperatorConstants.kDriverDeadbandY ? m_DriverController.getLeftX() * SwerveDriveConstants.kMaxDriveSpeed : 0.0, 
      () -> Math.abs(m_DriverController.getLeftY()) >= OperatorConstants.kDriverDeadbandX ? m_DriverController.getLeftY() * SwerveDriveConstants.kMaxDriveSpeed : 0.0,
      () -> Math.abs(m_DriverController.getRightX()) >= OperatorConstants.kDriverDeadbandTheta ? -m_DriverController.getRightX() * SwerveDriveConstants.MaxRotationSpeedCorrected : 0.0, // Switch back to commendted out 
      m_DriverController.leftBumper(),
      m_DriverController.leftTrigger());

  /*private final ArmSubsystem m_ArmSubsystem = 
    new ArmSubsystem();*/


  public RobotContainer() {
    configureBindings();
    setDefaultCommand();
  }

  private void configureBindings() {
    m_OperatorController.y().onTrue(new ElevatorCommand(m_ElevatorSubsystem, FieldConstants.kCoralBranchHeight_L4, true)); // TESTING 
    m_OperatorController.a().onTrue(new ElevatorCommand(m_ElevatorSubsystem, FieldConstants.kCoralBranchHeight_L1, true));
    m_OperatorController.x().onTrue(new ElevatorCommand(m_ElevatorSubsystem, FieldConstants.kCoralBranchHeight_L3, true));
    m_OperatorController.b().onTrue(new ElevatorCommand(m_ElevatorSubsystem, FieldConstants.kCoralBranchHeight_L2, true));
    m_DriverController.povLeft().onTrue(new EndEffectorCommand(m_EndEffectorSubsystem, EndEffectorConstants.kEffectorMotorSpeed, true));
    m_DriverController.povRight().onTrue(new EndEffectorCommand(m_EndEffectorSubsystem, EndEffectorConstants.kEffectorMotorSpeed * -1, false));
    m_DriverController.leftBumper().onTrue(new EndEffectorCommand(m_EndEffectorSubsystem, 0, false));
    m_OperatorController.povUp().onTrue(new ArmCommand(m_ArmSubsystem, 0.20)); // TESTING
    m_OperatorController.povDown().onTrue(new ArmCommand(m_ArmSubsystem, -0.20)); // TESTING
  }

  private void setDefaultCommand()
  {
    m_SwerveSubsystem.setDefaultCommand(joystickSwerve);
  }

  // PATHPLANNER IMPLEMNTATION TO GO HERE 
  public Command getAutonomousCommand() {
    return Autos.primaryAuto(m_SwerveSubsystem);
  }
}



