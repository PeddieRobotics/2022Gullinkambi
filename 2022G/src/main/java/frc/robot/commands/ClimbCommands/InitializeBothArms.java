package frc.robot.commands.ClimbCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

public class InitializeBothArms extends ParallelCommandGroup {
    public InitializeBothArms(){
        addCommands(new InitializeLeftArm(), new InitializeRightArm());
    }
}
