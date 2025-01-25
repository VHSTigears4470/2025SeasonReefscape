package frc.robot.commands.CoralCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CoralConstants.CORAL_ARM_STATE;
import frc.robot.subsystems.CoralSubsystem;

public class ToggleCoralArm extends Command {
    private static CoralSubsystem m_coralSub;
        
        public ToggleCoralArm(CoralSubsystem coralSub, CORAL_ARM_STATE armState) {
            m_coralSub = coralSub;
            m_coralArmState = armState;
            addRequirements(m_coralSub);
        }

        // Called when the command is initially scheduled.
        @Override
        public void initialize() {}

        // Called every time the scheduler runs while the command is scheduled.
        @Override
        public void execute(){ //runs every 20 milliseconds
            //if the arm is in the forward position, set it to the backward position
            if (m_coralArmState == ARM_STATE. && !m_coralSub.isAtDesiredPosition()) {
                //if the arm is not at the desired position, set the arm state to backward
                m_coralSub.setArmState(ARM_STATE.BACKWARD);
            }else if (!m_coralSub.isAtDesiredPosition()){
                m_coralSub.setArmState(ARM_STATE.FORWARD);
            }
        }

        // Called once the command ends or is interrupted.
        @Override
        public void end(boolean interrupted){
        //Stops the coralSub
         m_coralSub.stop();
        }

        // Returns true when the command should end. (never happens)
        @Override
        public boolean isFinished(){
            return false;
        }
}