package frc.robot.commands.CoralCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CoralConstants.CORAL_ARM_STATE;
import frc.robot.subsystems.CoralSubsystem;

public class ToggleCoralArm extends Command {
    private final CoralSubsystem m_coralSub;
    private CORAL_ARM_STATE m_coralArmState;
        
        public ToggleCoralArm(CoralSubsystem coralSub, CORAL_ARM_STATE armState) {
            m_coralSub = coralSub;
            m_coralArmState = armState;
        }

        // Called when the command is initially scheduled.
        @Override
        public void initialize() {}

        // Called every time the scheduler runs while the command is scheduled.
        @Override
        public void execute(){ //runs every 20 milliseconds
            //if the arm is in the forward position, set it to the backward position
            if (m_coralArmState == CORAL_ARM_STATE.FORWARD) 
                m_coralSub.setArmState(CORAL_ARM_STATE.BACKWARD);
            //else, if the arm is in the backward position, set it to the forward position
            else if (m_coralArmState == CORAL_ARM_STATE.BACKWARD)
                m_coralSub.setArmState(CORAL_ARM_STATE.FORWARD);
        }

        // Called once the command ends or is interrupted.
        @Override
        public void end(boolean interrupted){
        //  m_coralSub.stop();
        }

        // Returns true when the command should end.
        @Override
        public boolean isFinished(){
            return true;
        }
}