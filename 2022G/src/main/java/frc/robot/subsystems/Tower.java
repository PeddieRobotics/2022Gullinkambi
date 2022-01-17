package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants;
import frc.robot.utils.RobotMap;
//import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;

public class Tower extends SubsystemBase{

    private CANSparkMax towerBelt, towerRoller;
    private static Tower tower;
    //private AnalogInput topSensor0, middleSensor1, bottomSensor2, bottomSensor3;

    private DigitalInput topSensor0, middleSensor1, bottomSensor2, bottomSensor3;
    
    public static Tower getInstance(){
        if (tower == null) {
            tower = new Tower();
        }
        return tower;
    }
    
    public Tower() {
        setupTower();
        
        topSensor0 = new DigitalInput(0);
        middleSensor1 = new DigitalInput(1);
        bottomSensor2 = new DigitalInput(2);
        bottomSensor3 = new DigitalInput(3);
        
        /* 
        topSensor0 = new AnalogInput(0);
        middleSensor1 = new AnalogInput(1);
        bottomSensor2 = new AnalogInput(2);
        bottomSensor3 = new AnalogInput(3);
        */
    }

    private void setupTower() {
        towerRoller = new CANSparkMax(RobotMap.TOWER_ROLLER, MotorType.kBrushless);
        towerBelt = new CANSparkMax(RobotMap.TOWER_BELT, MotorType.kBrushless);
        towerRoller.setIdleMode(IdleMode.kBrake);
        towerBelt.setIdleMode(IdleMode.kBrake);
        towerRoller.setSmartCurrentLimit(Constants.MAX_TOWERROLLER_SPEED);
        towerBelt.setSmartCurrentLimit(Constants.MAX_TOWERBELT_SPEED);
    }

    public void reverseTower(double percent) { 
        //In the Egret code, the speed of the roller was reverse, so there's a difference between the signs for the speed of the roller and the belt
        runTowerRoller(percent);
        runTowerBelt(-percent);
    }

    public void runTowerRoller(double speed){
        towerRoller.set(-speed);
    }

    public void runTowerBelt(double speed){
        towerBelt.set(speed);
    }

    public void runTowerMotors(double beltSpeed, double rollerSpeed) {
        runTowerBelt(beltSpeed);
        runTowerRoller(rollerSpeed);
    }

    public void stopTower() {
        runTowerMotors(0.0, 0.0);
    }

    public boolean sensesBallBottom(){
        return (bottomSensor2.get() && bottomSensor3.get());
    }

    public boolean sensesBallMiddle(){
        return middleSensor1.get();
    } 

    public boolean sensesBallTop(){
        return topSensor0.get();
    } 

    /* Analog input stuff (just in case)
    public boolean sensesBallBottom(){
        return (bottomSensor2.getAverageVoltage() < 2.0);
    }

    public boolean sensesBallMiddle(){
        return (middleSensor1.getAverageVoltage() < 2.0);
    } 

    public boolean sensesBallTop(){
        return (topSensor0.getAverageVoltage() < 2.0);
    } 

    public double bottomSensorVoltage(){
        return bottomSensor2.getVoltage();
    }

    public double middleSensorVoltage(){
        return middleSensor1.getVoltage();
    } 

    public double topSensorVoltage(){
        return topSensor0.getVoltage();
    } 
    */
}