package frc.robot.commands.AlgaeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class ShootAlgae extends Command{
    private final AlgaeSubsystem m_algaeSub;

    public ShootAlgae  (AlgaeSubsystem algaeSub, IntakeSubsystem intakeSystem)
     m_algaeSub = algaeSub;
     addRequirements(algaeSub);
{
  
}
}
