package frc.robot.commands.AutoCommands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.TeleConstants;
import frc.robot.subsystems.DriveSubsystem;

public class DrivetoPosPose extends Command{
    private final DriveSubsystem m_driveSubsystem;

    private final double xSpd, ySpd, turnSpd;
    private final boolean fieldOriented;
    private Pose2d finalPose;
    private final double threshold = 0.05;
    private Transform2d translation;

    private final SlewRateLimiter xLimiter, yLimiter, turnLimiter;
    
    public DrivetoPosPose(DriveSubsystem driveSubsystem){
        m_driveSubsystem = driveSubsystem;

        xSpd =-0.3;
        ySpd = 0.0;
        turnSpd = 0.0;
        fieldOriented = false;

        translation = new Transform2d(1,0,new Rotation2d(0));
        this.xLimiter = new SlewRateLimiter(TeleConstants.k_MaxAccelerationMetersPerSecondSquared);
        this.yLimiter = new SlewRateLimiter(TeleConstants.k_MaxAccelerationMetersPerSecondSquared);
        this.turnLimiter = new SlewRateLimiter(TeleConstants.k_MaxAccelerationMetersPerSecondSquared);

        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize(){
        finalPose = m_driveSubsystem.getPose().plus(translation);
        SmartDashboard.putString("Drive Mode", "Default / Field Oriented");
    }

    @Override
    public void execute(){
        double xSpeed = OIConstants.k_driverXAxisInverted * xSpd;
        double ySpeed = OIConstants.k_driverYAxisInverted * ySpd;
        double turnSpeed = OIConstants.k_driverRotAxisInverted * turnSpd;

        xSpeed = Math.abs(xSpeed) > OIConstants.k_DriveDeadband ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > OIConstants.k_DriveDeadband ? ySpeed : 0.0;
        turnSpeed = Math.abs(turnSpeed) > OIConstants.k_DriveDeadband ? turnSpeed : 0.0;

        xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.k_MaxSpeedMetersPerSecond;
        ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.k_MaxSpeedMetersPerSecond;
        turnSpeed = turnLimiter.calculate(turnSpeed) * DriveConstants.k_MaxAngularSpeed;

        ChassisSpeeds chassisSpeeds;
        if(fieldOriented)
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turnSpeed, m_driveSubsystem.getRotation2d());
        else    
            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turnSpeed);

        SwerveModuleState[] moduleStates = DriveConstants.k_DriveKinematics.toSwerveModuleStates(chassisSpeeds);

        m_driveSubsystem.setModuleStates(moduleStates);
    }

    @Override
    public void end(boolean interrupted){
        m_driveSubsystem.stopModules();
    }
    
    @Override
    public boolean isFinished() {
        Pose2d distancePose = m_driveSubsystem.getPose().relativeTo(finalPose);
        double distance = Math.sqrt(distancePose.getX() * distancePose.getX() + distancePose.getY() * distancePose.getY());
        return distance < threshold;
    }
}
