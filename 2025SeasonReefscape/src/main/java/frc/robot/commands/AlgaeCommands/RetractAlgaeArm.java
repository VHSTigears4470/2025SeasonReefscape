package frc.robot.commands.AlgaeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AlgaeConstants.ALGAE_ARM_STATE;
import frc.robot.subsystems.AlgaeSubsystem;

public class RetractAlgaeArm extends Command {
    private final AlgaeSubsystem m_algaeSub;
        
        public RetractAlgaeArm(AlgaeSubsystem algaeSub) {
            m_algaeSub = algaeSub;
            addRequirements(m_algaeSub);
        }
       
        // Called when the command is initially scheduled.
        @Override
        public void initialize() {}

        // Called every time he scheduler runs while the command is scheduled.
        @Override
        public void execute() {
                m_algaeSub.setArmState(ALGAE_ARM_STATE.RAISED);
        }

        // Called once the command ends or is interrupted.
        @Override
        public void end(boolean interrupted) { //Should we stop the retractArm method or algaeSub before .stop? -- Vidur and Jay
        //I think we just add addRequirements(m_algaeSub) (already added) at the start, and then this just stops the rest -- Kenji
            m_algaeSub.stop();
        }
    
        // Returns true when the command should end.
        @Override
        public boolean isFinished() {
            return false;
        }
}