package frc.robot.commands.AlgaeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeSubsystem;

public class ShootAlgae extends Command{
    private final AlgaeSubsystem m_algaeSub;

    public ShootAlgae(AlgaeSubsystem algaeSub) {
        m_algaeSub = algaeSub;
        addRequirements(algaeSub);
    }

    @Override
    public void initialize() {}

    @Override 
    public void execute() {
        m_algaeSub.dispense();
    }

    @Override
    public void end(boolean interrupted) {
        m_algaeSub.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}