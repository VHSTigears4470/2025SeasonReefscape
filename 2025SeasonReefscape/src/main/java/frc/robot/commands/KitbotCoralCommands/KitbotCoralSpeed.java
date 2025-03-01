package frc.robot.commands.KitbotCoralCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.KitbotCoralSubsystem;

public class KitbotCoralSpeed extends Command {
    private final KitbotCoralSubsystem m_kitbotcoralSub;
    private final DoubleSupplier m_speed;

    //Constructor for TestAlgaeArm, also adds requirments so that this is the only command using algaeSub.
    public KitbotCoralSpeed(KitbotCoralSubsystem kitbotcoralSub, DoubleSupplier speed) {
        m_kitbotcoralSub = kitbotcoralSub;
        m_speed = speed;
        addRequirements(kitbotcoralSub);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time he scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_kitbotcoralSub.setSpeed(m_speed.getAsDouble());
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

