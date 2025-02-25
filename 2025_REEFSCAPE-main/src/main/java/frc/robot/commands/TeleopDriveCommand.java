
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.RobotConstants;
import frc.robot.subsystems.DriveSubsystem.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.math.filter.SlewRateLimiter;


// Auto needs
public class TeleopDriveCommand extends Command{
@SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

private final SwerveSubsystem m_SwerveSubsystem;
private final CommandPS4Controller m_driverController;

/*
private final SlewRateLimiter limit_X = new SlewRateLimiter(RobotConstants.kSlewRate_X);
private final SlewRateLimiter limit_Y = new SlewRateLimiter(RobotConstants.kSlewRate_Y);
private final SlewRateLimiter limit_Theta = new SlewRateLimiter(RobotConstants.kSlewRate_Theta);

double X_in;
double Y_in;
double Theta_in;
*/
public TeleopDriveCommand(SwerveSubsystem swerveSubsystem, CommandPS4Controller driveController) {
    this.m_SwerveSubsystem = swerveSubsystem;
    this.m_driverController = driveController;

    addRequirements(swerveSubsystem);
}

  @Override
  public void initialize() {
   // X_in = 0;
  //  Y_in = 0;
   // Theta_in = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
/* 
      X_in = m_driverController.getLeftX();
     Y_in = m_driverController.getLeftY();
      Theta_in = m_driverController.getRightX();

      m_SwerveSubsystem.driveBot(
      limit_X.calculate(X_in) * RobotConstants.kMaxLinearVelocity,
      limit_Y.calculate(Y_in) * RobotConstants.kMaxLinearVelocity,
      limit_Theta.calculate(Theta_in) * RobotConstants.kMaxRotationalVelocity, 

    false
     ); */
  }
}
