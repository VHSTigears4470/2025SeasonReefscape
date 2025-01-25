package frc.robot.commands.ClimbCommands;


public class RetractClimbArm extends Command {
    private static AlgaeSubsystem m_algaeSub;
        
        public RetractAlgaeArm(AlgaeSubsystem algaeSub, double desiredReferencePosition) {
            m_algaeSub = algaeSub;
        }
       
    
    @Override
    public void initialize() {}

    @Override
    public void execute() {
        m_algaeSub.setArmState(ALGAE_ARM_STATE.RAISED);
    }

    @Override
    public void end(boolean interrupted) { //Should we stop the retractArm method or algaeSub before .stop? -- Vidur and Jay
        //probably stop motors or something
        m_algaeSub.stop();
      }
    
      // Returns true when the command should end.
      //maybe need to change
      @Override
    public boolean isFinished() {
        return false;
    }
}
