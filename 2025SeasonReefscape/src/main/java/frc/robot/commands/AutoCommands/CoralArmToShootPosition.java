package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CoralConstants.CORAL_ARM_STATE;
import frc.robot.subsystems.CoralSubsystem;

public class CoralArmToShootPosition extends Command {
    private final CoralSubsystem m_coralSub;
    
    //Constructor for TestAlgaeArm, also adds requirments so that this is the only command using algaeSub.
    public CoralArmToShootPosition(CoralSubsystem coralSub) {
        m_coralSub = coralSub;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        //if the arm is in the forward position, set it to the backward position
        m_coralSub.setArmState(CORAL_ARM_STATE.BACKWARD);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute(){ //runs every 20 milliseconds
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted){
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished(){
        return m_coralSub.isAtDesiredPosition();
    }
}