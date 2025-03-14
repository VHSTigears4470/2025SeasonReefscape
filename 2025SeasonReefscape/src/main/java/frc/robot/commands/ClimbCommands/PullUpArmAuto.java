package frc.robot.commands.ClimbCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AlgaeConstants;
import frc.robot.Constants.ClimbConstants;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.ClimbSubsystem;

public class PullUpArmAuto extends Command {
    private final ClimbSubsystem m_climbSub;
    private final AlgaeSubsystem m_algaeSub;
    
    private boolean hasHitUpper;
    private boolean hasPassed;

    //Constructor for TestAlgaeArm, also adds requirments so that this is the only command using algaeSub.   
    public PullUpArmAuto(ClimbSubsystem climbSub, AlgaeSubsystem algaeSub) {
        m_climbSub = climbSub;
        m_algaeSub = algaeSub;
        addRequirements(m_climbSub, m_algaeSub);
    }
    
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        hasHitUpper = false;
        hasPassed = false;
        m_algaeSub.armForward();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // Keeps algae arm down
        if(m_algaeSub.getAlgaeArmEncoder() < AlgaeConstants.k_armDownGravityPosition) {
            // once the arm has hit down
            m_algaeSub.stopArmMotor();
        }

        // If it has not climbed above the buffer, keep climbing
        if(!hasHitUpper && m_climbSub.getClimbEncoder() > ClimbConstants.k_pullUpClimbPos) {
            if(!hasPassed) {
                m_climbSub.setArmSpeed(ClimbConstants.k_climbPullSpeed);
            } else {
                m_climbSub.setArmSpeed(-0.1);
            }
        } else {
            // Signals that it has hit the buffer
            hasHitUpper = true;
            hasPassed = true;
        }

        // If has hit the upper and climb is still within buffer, stop climb
        if(hasHitUpper && m_climbSub.getClimbEncoder() < ClimbConstants.k_pullUpClimbPos + ClimbConstants.k_positionBufferClimb) {
            m_climbSub.setArmSpeed(0);
        } else {
            // Signals that it has fallen out of the buffer
            hasHitUpper = false;
        }
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
