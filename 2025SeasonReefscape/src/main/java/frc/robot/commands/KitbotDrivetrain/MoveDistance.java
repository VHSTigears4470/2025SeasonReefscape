// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.KitbotDrivetrain;

import frc.robot.subsystems.KitbotDriveSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

/** An example command that moves the Kitbot Drive using arcade drive. */
public class MoveDistance extends Command {
  private final KitbotDriveSubsystem m_kitbotSub;
  private double m_distance, startingDistance;
  private double speed = 0.5; // need to check if 0 or not

  /**
   * Creates a new arcade drive command.
   *
   * @param subsystem The subsystem used by this command.
   * @param distance The distance in meters the robot should go (should be positive)
   * @param forward Whether or not the robot should be moving backwards 
   */
  public MoveDistance(KitbotDriveSubsystem kitbotSub, double distance, boolean forwards) {
    m_kitbotSub = kitbotSub;
    
    m_distance = Math.abs(distance);

    // modifies direction robot should go in
    if(!forwards) {
        speed *= -1;
    }
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(kitbotSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startingDistance = m_kitbotSub.getFrontRightEncoder();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_kitbotSub.arcadeDrive(0, -speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_kitbotSub.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(startingDistance - m_kitbotSub.getFrontRightEncoder()) > m_distance;
  }
}
