package frc.robot.commands.CoralCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralSubsystem;

public class TestCoralPosition extends Command {
    private final CoralSubsystem m_coralSub;
    private final double position;
    
    //Constructor for TestAlgaeArm, also adds requirments so that this is the only command using algaeSub.
    public TestCoralPosition(CoralSubsystem coralSub, double pos) {
        m_coralSub = coralSub;
        position = pos;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_coralSub.setTestReference(position);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute(){ //runs every 20 milliseconds
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted){
        m_coralSub.stopArmMotor();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished(){
        return false;
    }
}