package frc.robot.commands.ClimbCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

public class ExtendBothArms extends ParallelCommandGroup{
    public ExtendBothArms(){
        addCommands(new ExtendRightArm(), new ExtendLeftArm());
    }
}
