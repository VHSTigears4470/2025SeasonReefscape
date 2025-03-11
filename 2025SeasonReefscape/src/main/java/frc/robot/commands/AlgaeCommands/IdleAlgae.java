package frc.robot.commands.AlgaeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AlgaeConstants;
import frc.robot.subsystems.AlgaeSubsystem;

public class IdleAlgae extends Command {
    private AlgaeSubsystem m_algaeSub;
        
        public IdleAlgae(AlgaeSubsystem algaeSub) {
            m_algaeSub = algaeSub;
            addRequirements(m_algaeSub);
        }

        // Called when the command is initially scheduled.
         @Override
         public void initialize() {


         }

        // Called every time the scheduler runs while the command is scheduled.
        @Override
        public void execute(){
            m_algaeSub.stopIntakeMotor();
            if(m_algaeSub.getUpLimitSwitch() || Math.abs(m_algaeSub.getAlgaeArmEncoder()) < AlgaeConstants.k_algaePositionBuffer) {
              m_algaeSub.stopArmMotor();
            } else {
              m_algaeSub.holdIdle();
            }
              
        }

        // Called once the command ends or is interrupted.
        @Override
        public void end(boolean interrupted){
          m_algaeSub.stopIntakeMotor();
          m_algaeSub.stopArmMotor();
        }

        // Returns true when the command should end.
        @Override
        public boolean isFinished(){
          return false;
        }
}