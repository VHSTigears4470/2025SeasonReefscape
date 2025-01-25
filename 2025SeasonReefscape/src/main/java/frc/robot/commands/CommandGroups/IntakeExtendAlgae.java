package edu.wpi.first.wpilibj.examples.hatchbottraditional.commands;

import edu.wpi.first.wpilibj.examples.hatchbottraditional.Constants.AutoConstants;
import edu.wpi.first.wpilibj.examples.hatchbottraditional.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj.examples.hatchbottraditional.subsystems.HatchSubsystem;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/** A complex auto command that drives forward, releases a hatch, and then drives backward. */
public class IntakeExtendAlgae extends SequentialCommandGroup {
  /**
   * Creates a new ComplexAuto.
   *
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