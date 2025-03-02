package frc.robot.commands.ClimbCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimbConstants;
import frc.robot.subsystems.ClimbSubsystem;

public class PullUpArm extends Command {
    private final ClimbSubsystem m_climbSub;

    //Constructor for TestAlgaeArm, also adds requirments so that this is the only command using algaeSub.   
    public PullUpArm(ClimbSubsystem climbSub) {
        m_climbSub = climbSub;
        addRequirements(m_climbSub);
    }
    
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_climbSub.setArmSpeed(ClimbConstants.k_climbPullSpeed);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) { 
        m_climbSub.stop();    
    }
    
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true;
    }
}
