package frc.robot.commands.CommandGroups;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.SwerveDrivetrain.MoveDirection;
import frc.robot.commands.SwerveDrivetrain.SetTurnWheelAngles;
import frc.robot.subsystems.DriveSubsystem;

/** A complex auto command that drives forward, releases a hatch, and then drives backward. */
public class AutoMoveBasic extends SequentialCommandGroup {
  /**
   * @param AlgaeSubsystem The drive subsystem this command will run on
   */
  public AutoMoveBasic(DriveSubsystem swervSubsystem) {
    double xVal = 1;
    double yVal = 0;
    Transform2d translation = new Transform2d(new Translation2d(xVal, yVal), new Rotation2d(0));
    double angle = 0;
    addCommands(
            new SetTurnWheelAngles(swervSubsystem, angle),
            new WaitCommand(0.5),
            new MoveDirection(swervSubsystem, "Move Meters", translation)
    );
  }
}