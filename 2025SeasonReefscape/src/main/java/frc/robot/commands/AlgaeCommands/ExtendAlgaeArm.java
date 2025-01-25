package frc.robot.commands.AlgaeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AlgaeConstants.ALGAE_ARM_STATE;
import frc.robot.subsystems.AlgaeSubsystem;

public class ExtendAlgaeArm extends Command {
    private final AlgaeSubsystem m_algaeSub;
    private final double desiredReferencePosition;
    
    public ExtendAlgaeArm(AlgaeSubsystem algaeSub, double desiredReferencePosition) {
        m_algaeSub = algaeSub;
    
    }
}