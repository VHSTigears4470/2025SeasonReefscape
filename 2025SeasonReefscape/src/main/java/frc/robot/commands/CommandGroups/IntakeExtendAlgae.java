package frc.robot.commands.CommandGroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AlgaeCommands.ExtendAlgaeArm;
import frc.robot.commands.AlgaeCommands.IntakeAlgae;
import frc.robot.subsystems.AlgaeSubsystem;

/** A complex auto command that drives forward, releases a hatch, and then drives backward. */
public class IntakeExtendAlgae extends SequentialCommandGroup {
  /**
   * @param AlgaeSubsystem The drive subsystem this command will run on
   */
  public IntakeExtendAlgae(AlgaeSubsystem algaeSubsystem) {
    addCommands(
            new ParallelCommandGroup(
                new ExtendAlgaeArm(algaeSubsystem),
                new IntakeAlgae(algaeSubsystem)
            )
    );
  }
}