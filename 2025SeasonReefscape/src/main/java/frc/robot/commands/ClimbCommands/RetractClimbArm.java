package frc.robot.commands.ClimbCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbSubsystem;

public class RetractClimbArm extends Command {
    private final ClimbSubsystem m_climbSub;
        
    //Constructor for TestAlgaeArm, also adds requirments so that this is the only command using algaeSub.
    public RetractClimbArm(ClimbSubsystem climbSub) {
        m_climbSub = climbSub;
    }
       
    // Called once the command ends or is interrupted.
    @Override
        public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_climbSub.setArmSpeed(-0.25);
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
