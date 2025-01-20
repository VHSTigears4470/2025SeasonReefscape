package frc.robot.commands.SwerveDrivetrain;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class TestSetPosCommand extends Command {
    private final DriveSubsystem swerveSubsystem;
    private final double val;

    /**
     * Contructs a Command to control the swerve via joystick
     * @param swerveSubsystem subsystem that controls the swerve
     * @param val Position to rotate the wheels to (in radians)
     */
    public TestSetPosCommand(DriveSubsystem swerveSubsystem, double val) {
        this.swerveSubsystem = swerveSubsystem;
        this.val = val;
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        SmartDashboard.putString("Drive Mode", "Turning Motors"); // Helps understand which command swerve drive is using
    }

    @Override
    public void execute() {
        swerveSubsystem.testTurnMotors(val);
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
