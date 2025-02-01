package frc.robot.commands.AlgaeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AlgaeConstants.ALGAE_ARM_STATE;
import frc.robot.subsystems.AlgaeSubsystem;

public class RetractAlgaeArm extends Command {
    private final AlgaeSubsystem m_algaeSub;
        
        //creates a new RetractAlgae arm with the user's parameters, as well as making this the only command that is using the passed algae subsystem
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
        public void end(boolean interrupted) { 
            m_algaeSub.stop();
        }
    
        // Returns true when the command should end.
        @Override
        public boolean isFinished() {
            return false;
        }
}