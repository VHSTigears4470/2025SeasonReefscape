package frc.robot.commands.AlgaeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AlgaeConstants.ALGAE_ARM_STATE;
import frc.robot.subsystems.AlgaeSubsystem;

public class IdleAlgae extends Command {
    private AlgaeSubsystem m_algaeSub;
        
        public IdleAlgae(AlgaeSubsystem algaeSub) {
            m_algaeSub = algaeSub;
            addRequirements(m_algaeSub);
        }

        // Called when the command is initially scheduled.
         @Override
         public void initialize() {}

        // Called every time the scheduler runs while the command is scheduled.
        @Override
        public void execute(){
              if(m_algaeSub.getStowArmWhenIdle()) {
                m_algaeSub.setArmState(ALGAE_ARM_STATE.STOWED);
                m_algaeSub.stopIntakeMotor();
              } else {
                m_algaeSub.setArmState(ALGAE_ARM_STATE.HOLDING);
                m_algaeSub.hold();
              }
              
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