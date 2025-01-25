package frc.robot.commands.ClimbCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimbConstants.CLIMB_STATE;
import frc.robot.subsystems.ClimbSubsystem;

public class ExtendClimbArm extends Command {
    private static ClimbSubsystem m_climbSub;
        
        public ExtendClimbArm(ClimbSubsystem climbSub) {
            m_climbSub = climbSub;
            addRequirements(m_climbSub);//might not need? - Kenji
        } 
    
        @Override
        public void initialize() {}

        @Override
        public void execute() {
            m_climbSub.setArmSpeed(0.25);
        }

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
