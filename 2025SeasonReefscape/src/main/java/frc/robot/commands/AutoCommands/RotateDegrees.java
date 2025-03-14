package frc.robot.commands.AutoCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class RotateDegrees extends Command{
    private final DriveSubsystem m_driveSubsystem;
    private final Rotation2d m_rotation;
    private final double threshold = 1;
    private final double rotationSpeed;
    private Rotation2d m_goal;
    
    public RotateDegrees(DriveSubsystem driveSubsystem, Rotation2d rotation, boolean clockwise){
        m_driveSubsystem = driveSubsystem;
        m_rotation = rotation;
        if(clockwise) {
            rotationSpeed = 0.5;
        } else {
            rotationSpeed = -0.5;
        }
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize(){
        System.out.println("l");
        System.out.println("l");
        System.out.println("l");
        System.out.println("l");
        System.out.println("l");
        System.out.println("l");
        System.out.println("\n\n\n\n\nStart : " + m_driveSubsystem.getRotation2d().getDegrees());
        m_goal = m_driveSubsystem.getRotation2d().plus(m_rotation);
    }

    @Override
    public void execute(){
        double speed = rotationSpeed;
        double differenceAngle = m_goal.minus(m_driveSubsystem.getRotation2d()).getDegrees();
        if(Math.abs(differenceAngle) < 10) { 
            speed = Math.signum(rotationSpeed) * 0.25;
        }
        m_driveSubsystem.drive(0, 0, speed, false, "Rotate In Place");
    }

    @Override
    public void end(boolean interrupted){
        m_driveSubsystem.stopModules();
    }
    
    @Override
    public boolean isFinished() {
        double differenceAngle = m_goal.minus(m_driveSubsystem.getRotation2d()).getDegrees();
        System.out.println(m_goal.getDegrees() + " : " + differenceAngle + " : " + m_driveSubsystem.getRotation2d().getDegrees());
        return Math.abs(differenceAngle) < threshold;
    }
}
