package frc.robot.commands.AlgaeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AlgaeConstants.ALGAE_ARM_STATE;
import frc.robot.subsystems.AlgaeSubsystem;

public class ExtendAlgaeArm extends Command {
    private AlgaeSubsystem m_algaeSub;
        
        public ExtendAlgaeArm(AlgaeSubsystem algaeSub) {
            m_algaeSub = algaeSub;
            addRequirements(m_algaeSub);
        }

        // Called when the command is initially scheduled.
         @Override
         public void initialize() {}

        // Called every time the scheduler runs while the command is scheduled.
        @Override
          public void execute(){
              m_algaeSub.setArmState(ALGAE_ARM_STATE.LOWERED);
        }

        // Called once the command ends or is interrupted.
        @Override
        public void end(boolean interrupted){}

        // Returns true when the command should end.
        @Override
        public boolean isFinished(){
        return false;
        }
}