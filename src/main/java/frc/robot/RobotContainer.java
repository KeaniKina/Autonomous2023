package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.Balance;
import frc.robot.commands.DriverControl;
import frc.robot.commands.IncrementBalanceCommand;
import frc.robot.commands.MovementCommands.DriveBackwardCommand;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DriverControl;
import edu.wpi.first.wpilibj.XboxController;

public class RobotContainer {

  private final SwerveSubsystem swerve = new SwerveSubsystem();
  private final XboxController m_Controller =
      new XboxController(0); 

  public RobotContainer() {
    swerve.setDefaultCommand(new DriverControl(swerve, 
    () -> -m_Controller.getLeftY(), 
    () -> -m_Controller.getLeftX(),
    () -> -m_Controller.getRightX(), 
    () -> m_Controller.getRightBumper()));

    configureBindings();
  }

  private void configureBindings() {
  }

  public Command getAutonomousCommand() {
    return new Balance(swerve); 
  }
}
