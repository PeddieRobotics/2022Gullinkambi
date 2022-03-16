package frc.robot.commands.ClimbCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

public class RetractBothArms extends ParallelCommandGroup{
    public RetractBothArms(){
        addCommands(new RetractRightArm(), new RetractLeftArm());
    }
}
