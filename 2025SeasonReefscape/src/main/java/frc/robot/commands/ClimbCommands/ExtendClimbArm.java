package frc.robot.commands.ClimbCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimbConstants.CLIMB_STATE;
import frc.robot.subsystems.ClimbSubsystem;

public class ExtendClimbArm extends Command {
    private static ClimbSubsystem m_climbSub;
        
        public ExtendClimbArm(ClimbSubsystem climbSub) {
            m_climbSub = climbSub;
        } 
    
    @Override
    public void initialize() {}

    @Override
    public void execute() {
        m_climbSub.setClimbArmState(CLIMB_STATE.DOWN);
    }

    @Override
    public void end(boolean interrupted) { //Should we stop the retractArm method or algaeSub before .stop? -- Vidur and Jay
        //probably stop motors or something
        m_climbSub.stop();
      }
    
      // Returns true when the command should end.
      //maybe need to change
      @Override
    public boolean isFinished() {
        return false;
    }
}
