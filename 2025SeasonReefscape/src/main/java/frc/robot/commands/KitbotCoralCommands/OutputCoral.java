package frc.robot.commands.KitbotCoralCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.KitbotCoralSubsystem;

public class OutputCoral extends Command {
    private final KitbotCoralSubsystem m_kitbotcoralSub;

    public OutputCoral(KitbotCoralSubsystem kitbotcoralSub) {
        m_kitbotcoralSub = kitbotcoralSub;
        addRequirements(kitbotcoralSub);
    }
    

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time he scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_kitbotcoralSub.intake();
    } 
    
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_kitbotcoralSub.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}

