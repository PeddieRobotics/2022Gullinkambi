package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.RobotMap;
import frc.robot.utils.Constants;

public class Hopper extends SubsystemBase {
    private static Hopper hopper;
    private CANSparkMax hopperBelt, hopperRollerLeft, hopperRollerRight;

    public Hopper(){
       hopperBelt = new CANSparkMax(RobotMap.HOPPER_BELT, MotorType.kBrushless);
       hopperRollerLeft = new CANSparkMax(RobotMap.HOPPER_ROLLER_LEFT, MotorType.kBrushless);
       hopperRollerRight = new CANSparkMax(RobotMap.HOPPER_ROLLER_RIGHT, MotorType.kBrushless);
    }

    public static Hopper getInstance(){
        if (hopper == null){
            hopper = new Hopper();
        }

        return hopper;
    }

    public void runHopper(double hopperRollerLeftSpeed, double hopperRollerRightSpeed, double hopperBeltSpeed){
        hopperBelt.set(hopperBeltSpeed); //the speed input needs a multiplier
        hopperRollerLeft.set(hopperRollerLeftSpeed);
        hopperRollerRight.set(hopperRollerRightSpeed);
    }

    public void stopHopper(){
       runHopper(0,0,0);
    }

    public void reverseHopper(double hopperRollerLeftSpeed, double hopperRollerRightSpeed, double hopperBeltSpeed){
        runHopper(-hopperRollerLeftSpeed, -hopperRollerRightSpeed, -hopperBeltSpeed);
    }


    public double getBeltSpeed(){
        return(hopperBelt.get());
    }

    public double getRollerLeftSpeed(){
        return(hopperRollerLeft.get());
    }

    public double getRollerRightSpeed(){
        return(hopperRollerRight.get());
    }

    public void putSmartDashboard(){
        SmartDashboard.putNumber("Hopper Belt Speed", getBeltSpeed());
        SmartDashboard.putNumber("Hopper Left Roller Speed", getRollerLeftSpeed());
        SmartDashboard.putNumber("Hopper Right Roller Speed", getRollerRightSpeed());
    }
}

