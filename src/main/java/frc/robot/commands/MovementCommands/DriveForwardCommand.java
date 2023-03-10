package frc.robot.commands.MovementCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;

public class DriveForwardCommand extends CommandBase{
    private final SwerveSubsystem swerve;
    private double desiredEnc; 

    public DriveForwardCommand(SwerveSubsystem newSwerve, double newDesiredEnc){
        swerve = newSwerve;
        desiredEnc = newDesiredEnc; 

        addRequirements(swerve);
    }

    @Override
    public void initialize(){
        swerve.resetEnc();
    }

    @Override
    public void execute(){
        swerve.driveForward();
    }

    @Override
    public void end(boolean interrupted){
        swerve.stopModules();
    }

    @Override
    public boolean isFinished(){
        return swerve.getEnc() > desiredEnc;
    }

}
