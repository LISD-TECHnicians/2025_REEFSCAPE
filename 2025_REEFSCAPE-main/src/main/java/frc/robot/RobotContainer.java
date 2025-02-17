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
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.Constants.RobotConstants.SwerveDriveConstants;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {

  private final CommandPS4Controller m_DriverController =
    new CommandPS4Controller(OperatorConstants.kDriverControllerPort);
  
  private final CommandPS4Controller m_OperatorController = 
    new CommandPS4Controller(OperatorConstants.kOperatorControllerPort);

  private final ElevatorSubsystem m_ElevatorSubsystem = 
    new ElevatorSubsystem();

    private final SwerveSubsystem m_SwerveSubsystem = new SwerveSubsystem();

private final SwerveCmd joystickSwerve = new SwerveCmd(
      m_SwerveSubsystem, 
      () -> Math.abs(m_DriverController.getLeftX()) >= OperatorConstants.kDriverDeadbandY ? m_DriverController.getLeftX() * SwerveDriveConstants.kMaxDriveSpeed : 0.0, 
      () -> Math.abs(m_DriverController.getLeftY()) >= OperatorConstants.kDriverDeadbandX ? m_DriverController.getLeftY() * SwerveDriveConstants.kMaxDriveSpeed : 0.0,
      () -> Math.abs(m_DriverController.getRightX()) >= OperatorConstants.kDriverDeadbandTheta ? -m_DriverController.getRightX() * SwerveDriveConstants.MaxRotationSpeedCorrected : 0.0, // Switch back to commendted out 
      m_DriverController.L1(),
      m_DriverController.L2());

  /*private final ArmSubsystem m_ArmSubsystem = 
    new ArmSubsystem();*/


  public RobotContainer() {
    configureBindings();
    setDefaultCommand();
  }

  private void configureBindings() {
    m_OperatorController.triangle().onTrue(new ElevatorCommand(m_ElevatorSubsystem, FieldConstants.kCoralBranchHeight_L4)); // TESTING 
    m_OperatorController.cross().onTrue(new ElevatorCommand(m_ElevatorSubsystem, FieldConstants.kCoralBranchHeight_L1));
    m_OperatorController.square().onTrue(new ElevatorCommand(m_ElevatorSubsystem, FieldConstants.kCoralBranchHeight_L3));
    m_OperatorController.circle().onTrue(new ElevatorCommand(m_ElevatorSubsystem, FieldConstants.kCoralBranchHeight_L2));
  
    //m_OperatorController.povUp().onTrue(new ArmCommand(m_ArmSubsystem, 0.20, 0.20)); // TESTING
    //m_OperatorController.povDown().onTrue(new ArmCommand(m_ArmSubsystem, -.20, -.20)); // TESTING
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



