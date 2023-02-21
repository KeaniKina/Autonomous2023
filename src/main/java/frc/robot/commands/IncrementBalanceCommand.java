package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.commands.MovementCommands.DriveForwardCommand;
import frc.robot.commands.MovementCommands.DriveBackwardCommand;

public class IncrementBalanceCommand extends CommandBase {
  private final SwerveSubsystem swerve; 
  private Timer timer; 
  private Timer autoTimer; 
  private double prevPitch; 
  private double currentPitch; 

  public IncrementBalanceCommand(SwerveSubsystem newSwerve) {
    swerve = newSwerve; 
    timer = new Timer(); 
    autoTimer = new Timer(); 

    addRequirements(swerve);
  }

  @Override
  public void initialize() {
  timer.reset();
  timer.start();
  autoTimer.start();

  prevPitch = 0; 
  }

  @Override
  public void execute() {
    SmartDashboard.putNumber("Timer", timer.get());
    SmartDashboard.putString("Current Command", getName());

    if (timer.get() >= 1) {
      swerve.driveBackward(); 
      timer.reset();
    } 

    prevPitch = currentPitch; 
    currentPitch = swerve.getRoll();
  }

  @Override
  public void end(boolean interrupted) {
    swerve.lock(); 
    SmartDashboard.putString("Current Command", "");
  }

  @Override
  public boolean isFinished() {
    /*navx reads drop or autonomous period ends  */
    return Math.abs(currentPitch - prevPitch) >= 5 || autoTimer.get() >= 14.5; //change timer bool later  if need
  }
}
