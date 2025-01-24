package frc.robot.commands.AlgaeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AlgaeConstants.ALGAE_ARM_STATE;
import frc.robot.Constants.IntakeConstants.ARM_STATE;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.CoralSubsystem;

public class ShootAlgae extends Command{
    private final AlgaeSubsystem m_algaeSub;
    private final ALGAE_ARM_STATE desiredReferencePosition;

    public ShootAlgae (AlgaeSubsystem algaeSub) {
        m_algaeSub = algaeSub;
        addRequirements(algaeSub);
    }
}
