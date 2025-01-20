package frc.robot.commands.SwerveDrivetrain;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class TestDrivingMotors extends Command {
    private final DriveSubsystem swerveSubsystem;
    private final double speed;

    /**
     * Contructs a Command to continously rotate only the wheels
     * @param swerveSubsystem subsystem that controls the swerve
     * @param moveClockwise whether or not the robot wheels should move clockwise
     */
    public TestDrivingMotors(DriveSubsystem swerveSubsystem, double speed) {
        this.swerveSubsystem = swerveSubsystem;
        this.speed = speed;
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        SmartDashboard.putString("Drive Mode", "Driving Wheels"); // Helps understand which command swerve drive is using
    }

    @Override
    public void execute() {
        swerveSubsystem.testDriveMotors(speed);
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
