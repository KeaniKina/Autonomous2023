package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;

public class Balance extends CommandBase {
  private final SwerveSubsystem swerve;
  private final Timer timer, autoTimer; 
  private int counter;

  public Balance(SwerveSubsystem newSwerve) {
    swerve = newSwerve;
    timer = new Timer();
    autoTimer = new Timer(); 

    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    autoTimer.start();
    swerve.resetEnc();
    counter = 0;
  }

  @Override
  public void execute() {
    SmartDashboard.putString("Current Command", getName());
    SmartDashboard.putNumber("Balance Counter", counter);
    SmartDashboard.putNumber("Auto Timer", autoTimer.get());
    SmartDashboard.putNumber("Timer", timer.get());

    switch (counter) {
      case 0: //drive until -12 deg
        if (swerve.getRoll() <= -12) {
          swerve.stopModules();
          counter++; 
        } else {
          swerve.driveBackward();
        }
      break; 

      case 1: // drive a little
        if (swerve.getEnc() < -20) {
          swerve.stopModules();
          counter++;
        } else {
          swerve.driveBackward();
        }

      break;

      case 2: // wait and check if roll goes below 10??? deg and go back to case 1 if not 
      timer.start();
      if (timer.get() > 2) { //wait 2 sec 

        //if level, stop 
        if (swerve.getRoll() > -10) {
          counter++;
        } else {
          counter = 0;
        }

      }

      break;
    }
  }

  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putString("Current Command", "");
    swerve.stopModules();
    swerve.lock();
  }

  @Override
  public boolean isFinished() {
    return counter == 3 || autoTimer.get() >= 14.5;
  }
}
