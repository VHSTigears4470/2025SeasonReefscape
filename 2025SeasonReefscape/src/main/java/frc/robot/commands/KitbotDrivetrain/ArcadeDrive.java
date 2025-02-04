// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.KitbotDrivetrain;

import frc.robot.subsystems.KitbotDriveSubsystem;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;

/** An example command that moves the Kitbot Drive using arcade drive. */
public class ArcadeDrive extends Command {
  private final KitbotDriveSubsystem m_kitbotSub;
  private final DoubleSupplier m_xSpeed, m_zRotation;
  private final double multiplier = 0.5;

  /**
   * Creates a new arcade drive command.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ArcadeDrive(KitbotDriveSubsystem kitbotSub, DoubleSupplier xSpeed, DoubleSupplier zRotation) {
    m_kitbotSub = kitbotSub;
    m_xSpeed = xSpeed;
    m_zRotation = zRotation;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(kitbotSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_kitbotSub.arcadeDrive(m_xSpeed.getAsDouble() * multiplier, m_zRotation.getAsDouble() * multiplier);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_kitbotSub.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
