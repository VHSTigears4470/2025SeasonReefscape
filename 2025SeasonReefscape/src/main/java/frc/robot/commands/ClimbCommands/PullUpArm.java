package frc.robot.commands.ClimbCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AlgaeConstants;
import frc.robot.Constants.ClimbConstants;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.ClimbSubsystem;

public class PullUpArm extends Command {
    private final ClimbSubsystem m_climbSub;
    private final AlgaeSubsystem m_algaeSub;

    //Constructor for TestAlgaeArm, also adds requirments so that this is the only command using algaeSub.   
    public PullUpArm(ClimbSubsystem climbSub, AlgaeSubsystem algaeSub) {
        m_climbSub = climbSub;
        m_algaeSub = algaeSub;
        addRequirements(m_climbSub, m_algaeSub);
    }
    
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_algaeSub.armForward();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if(m_algaeSub.getAlgaeArmEncoder() < AlgaeConstants.k_armDownGravityPosition) {
            // once the arm has hit down
            m_algaeSub.stopArmMotor();
        }
        m_climbSub.setArmSpeed(ClimbConstants.k_climbPullSpeed);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) { 
        m_climbSub.stop();    
        m_algaeSub.stopArmMotor();
    }
    
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
