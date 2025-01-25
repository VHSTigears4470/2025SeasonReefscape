package frc.robot.commands.CoralCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeSubsystem;

public class IntakeCoral extends Command {
    private final CoralSubsystem m_coralSub;

    public IntakeCoral(CoralSubsystem coralSub) {
        m_coralSub = coralSub;
        addRequirements(m_coralSub);
    }
    

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time he scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        //Intakes the coral
        m_coralSub.intake();
    } 
    
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        //stops the coral subsystem
        m_coralSub.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}