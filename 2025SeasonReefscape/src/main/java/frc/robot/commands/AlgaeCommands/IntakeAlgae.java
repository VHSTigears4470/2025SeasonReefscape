package frc.robot.commands.AlgaeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AlgaeConstants.ALGAE_ARM_STATE;
import frc.robot.subsystems.AlgaeSubsystem;

public class IntakeAlgae extends Command {
    private final AlgaeSubsystem m_algaeSub;

    public IntakeAlgae(AlgaeSubsystem algaeSub) {
        m_algaeSub = algaeSub;
        addRequirements(algaeSub);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_algaeSub.setStowArmWhenIdle(false);
    }

    // Called every time he scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_algaeSub.setArmState(ALGAE_ARM_STATE.DOWN);
        m_algaeSub.intake();
    } 
    
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
