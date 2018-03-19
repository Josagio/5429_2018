/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team5429.robot;

import edu.wpi.first.wpilibj.IterativeRobot;


import edu.wpi.first.wpilibj.Timer;


import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.awt.geom.CubicCurve2D;

import com.ctre.CANTalon;

import edu.wpi.first.wpilibj.IterativeRobot;


import edu.wpi.first.wpilibj.Joystick;


import edu.wpi.first.wpilibj.Joystick.AxisType;


import edu.wpi.first.wpilibj.RobotDrive;


import edu.wpi.first.wpilibj.RobotDrive.MotorType;


import com.ctre.phoenix.*;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.motorcontrol.can.*;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;



import edu.wpi.first.wpilibj.CameraServer;


import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.GyroBase;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.SensorBase;
import edu.wpi.first.wpilibj.GyroBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;


/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
    
	//used to select what we are targeting
		final String defaultAuto = "Default";
		final String SwitchAuto = "SwitchAuto";
	    final String ScaleAuto = "ScaleAuto";
	    final String BaseAuto = "BaseAuto";
	    final String PreloadAuto = "PreloadAuto";
	    
	    //used to select what position the robot is in
	    final String leftPosition = "leftPosition";
	    final String rightPosition = "rightPosition";
	    final String centerPosition = "centerPosition";
	    
	    final int zeroSeconds = 0;
	    final int twoSeconds = 2;
	    final int threeSeconds = 3;
	    final int fiveSeconds = 5;
	    final int sixSeconds = 6;
		
	    ADXRS450_Gyro gyro = new ADXRS450_Gyro();
	    
	    //final String resetEncoder = "resetEncoder";
	    //final String liveEncoder = "liveEncoder" ;
	    
	    
	    String positionSelected;
	    String autoSelected;
	    int delaySelected;
	    //String autoSelectedButton;
	    SendableChooser chooser;
	    SendableChooser position;
	    SendableChooser delay;
    
    WPI_TalonSRX _frontLeftMotor = new WPI_TalonSRX(0); 		/* device IDs here (1 of 2) */
    WPI_TalonSRX _rearLeftMotor = new WPI_TalonSRX(1);
    WPI_TalonSRX _frontRightMotor = new WPI_TalonSRX(2);
    WPI_TalonSRX _rearRightMotor = new WPI_TalonSRX(3);
	//CameraServer.StartAutomaticCapture();
	/* extra talons for six motor drives */
	WPI_TalonSRX _elevator = new WPI_TalonSRX(4);
	WPI_VictorSPX _intakeLeft = new WPI_VictorSPX(7);
	WPI_VictorSPX _intakeRight = new WPI_VictorSPX(8);
	WPI_TalonSRX _climb = new WPI_TalonSRX(5);
	
	RobotDrive _drive = new RobotDrive(_frontLeftMotor, _rearLeftMotor, _frontRightMotor, _rearRightMotor);
	Joystick _joy = new Joystick(0);
	Joystick _joy2 = new Joystick(1);
	//Joystick _xBoxController = new Joystick(0);
	Joystick _partnerController = new Joystick(2);
	CameraServer server = CameraServer.getInstance();
	
	//server.setQuality(50);
//	stry.startAutomaticCapture();
	
	/**
	 * Simple thread to plot the sensor velocity
	 */
	PlotThread _plotThread;
	
	//Shovel Knight
	String Shovel_Knight;
	
	
	// Autonomous Variables
	int autoCounter = 0;
	int autoState = 0;
	
	
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
	
	
    public void robotInit() {
    	chooser = new SendableChooser();
        chooser.addDefault("Default Auto", defaultAuto);
        chooser.addObject("Switch Auto", SwitchAuto);
        chooser.addObject("Scale Auto", ScaleAuto);
        chooser.addObject("BaseAuto", BaseAuto);
        chooser.addObject("Preload Auto", PreloadAuto);
        
        position = new SendableChooser();
        position.addDefault("Default position", null);
        position.addObject("Left Position", leftPosition);        
        position.addObject("Right Position", rightPosition);
        position.addObject("Center Position", centerPosition);
        
        delay = new SendableChooser();
        delay.addDefault("Zero", zeroSeconds);
        delay.addObject("Two", twoSeconds);
        delay.addObject("Three", threeSeconds);
        delay.addObject("five", fiveSeconds);
        delay.addObject("Six", sixSeconds);
        
        Shovel_Knight = "Robot has initiated!";
        
        SmartDashboard.putData("Auto choices", chooser);
        SmartDashboard.putData("Robot Position", position);
        SmartDashboard.putData("Auto delay", delay);
        SmartDashboard.putString("Shovel Knight:", Shovel_Knight);
        
    	server.startAutomaticCapture();
    	
    	//Sensor automatic capturing
    	gyro.reset();
        gyro.calibrate();
    	
    	
    	
    	/* the Talons on the left-side of my robot needs to drive reverse(red) to move robot forward.
    	 * Since _leftSlave just follows frontLeftMotor, no need to invert it anywhere. */
    	_drive.setInvertedMotor(MotorType.kFrontLeft, true);
    	_drive.setInvertedMotor(MotorType.kRearLeft, true);
    
    	
    	
    	/*
		 * new frame every 1ms, since this is a test project use up as much
		 * bandwidth as possible for the purpose of this test.
		 */
		_elevator.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 1, 10);
		_elevator.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);

		_climb.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 1, 10);
		_climb.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);

		/* fire the plotter */
		new Thread(_plotThread).start();
    	
    	_elevator.setSelectedSensorPosition(0, 0, 0);
    	_climb.setSelectedSensorPosition(0, 0, 0);
    }
    
	/**
	 * This autonomous (along with the chooser code above) shows how to select between different autonomous modes
	 * using the dashboard. The sendable chooser code works with the Java SmartDashboard. If you prefer the LabVIEW
	 * Dashboard, remove all of the chooser code and uncomment the getString line to get the auto name from the text box
	 * below the Gyro
	 *
	 * You can add additional auto modes by adding additional comparisons to the switch structure below with additional strings.
	 * If using the SendableChooser make sure to add them to the chooser code above as well.
	 */
    public void autonomousInit() {
    	
    	autoCounter = 0;
    	autoState = 0;
    	float Kp = -0.1f;
    	
    	
    	autoSelected = (String) chooser.getSelected();
    	positionSelected = (String) position.getSelected();
    	delaySelected = (int) delay.getSelected();
    	
    	Shovel_Knight = "We are aiming for " + autoSelected + "and the robot is placed in " + positionSelected;
    	
    	
//		autoSelected = SmartDashboard.getString("Auto Selector", defaultAuto);
		System.out.println("Auto selected: " + autoSelected);
		System.out.println("Positon selected:" + positionSelected);
		SmartDashboard.putString("position choice:", (String) position.getSelected());
		SmartDashboard.putString("Shovel Knight:", Shovel_Knight);
		
    }
    
    public void Run_Center_Right_Switch(){
      	NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    	NetworkTableEntry tx = table.getEntry("tx");
    	NetworkTableEntry ty = table.getEntry("ty");
    	NetworkTableEntry ta = table.getEntry("ta");
    	double x = tx.getDouble(0);
    	double y = ty.getDouble(0);
    	double area = ta.getDouble(0);
    	float Kp = -0.1f;
    	double error = x;
    	double steering_adjust = x * Kp;
    	switch(autoState)
 	   {
    	    case 0:
    	    	table.getEntry("ledMode").setNumber(1);
    	    	_drive.setInvertedMotor(MotorType.kFrontLeft, false);
 	   	   		_drive.setInvertedMotor(MotorType.kRearLeft, false);
 	   	   		_drive.drive(-.10, 0);
 	   	   		if(autoCounter >= 40){
 	   	   			autoState = 1;
 	   	   			autoCounter = 0;
 	   	   		}
    	    	break;
 	   		case 1:
 	   		table.getEntry("ledMode").setNumber(0);
 	   			error = 0;
 	   			_drive.setInvertedMotor(MotorType.kFrontLeft, false);
 	   	   		_drive.setInvertedMotor(MotorType.kRearLeft, false);
 	   			_drive.drive(-.5, steering_adjust * -1);//test to see if needed to be a negative
 	   			//_drive.arcadeDrive(0, steering_adjust);
 	   			if(autoCounter >= 2000 || area > 15){
 	   			autoState = 2;
 	   			autoCounter = 0;
 	   			}
 	   			break;
 	   		case 2:
 	   	        /*
 	   	         * _drive.setInvertedMotor(MotorType.kFrontLeft, true);
 	   	   		   _drive.setInvertedMotor(MotorType.kRearLeft, true);
 	   			   _drive.drive(0, 0);		
 	   	         */
 	    		_intakeRight.set(1);
 	    		_intakeLeft.set(1);
 	   			_drive.drive(-.45, 0);
 	   		if(autoCounter >= 30){
 	   			autoState = 3;
 	   			autoCounter = 0;
 	   			}
 	   			break;
 	   		case 3:
 	    		_intakeRight.set(0);
 	    		_intakeLeft.set(0);
 	   			_drive.drive(0, 0);
 	   			if(autoCounter >= 10){
 	   				autoState = 4;
 	   				autoCounter = 0;
 	   			}
 	   		case 4:
 	   			_drive.drive(.30,0);
 	   			if(autoCounter >= 45){
 	   				autoState = 5;
 	   				autoCounter = 0;
 	   			}
 	   			break;
 	   		case 5:
 	   			_drive.drive(0, 0);
 	   			if(autoCounter >2.5){
 	   				autoState = 6;
 	   				autoCounter = 0;
 	   			}
 	   			break;
 	   		case 6:
 	   			while(gyro.getAngle() < 21.5){
 	   			_drive .drive(.4, -1);
 	   			}
 	   			_drive.drive(0, 0);
 	   			if(autoCounter > 2.5){
 	   				autoState = 7;
 	   				autoCounter = 0;
 	   			}
 	   			break;
 	   		case 7:
 	   			_drive.drive(-.6, 0);
 	   			if(autoCounter > 44){
 	   				autoState = 8;
 	   				autoCounter = 0;
 	   			}
 	   			break;
 	   		case 8:
 	   			_drive.drive(0, 0);
 	   			if(autoCounter > 2.5){
 	   				autoState = 9;
 	   				autoCounter = 0;
 	   			}
 	   			break;
 	   		case 9:
 	   			while(_elevator.getSelectedSensorPosition(0) <= 17550){
 	   				_elevator.set(ControlMode.PercentOutput, -100);
 	   			}
 	   			_elevator.set(0);
 	   			autoState = 10;
 	   			autoCounter = 0;
 	   			break;
 	   		case 10:
 	   			_drive.drive(-.35, 0);
 	   			if(autoCounter > 42){
 	   				_drive.drive(0, 0);
 	   				autoState = 11;
 	   				autoCounter = 0;
 	   			}
 	   				
 	   			break;
 	   		case 11:
 	    		_intakeRight.set(-1);
 	    		_intakeLeft.set(-1);
 	    		_drive.drive(.3, 0);
 	    		if(autoCounter > 15){
 	 	    	   autoCounter = 0;
 	 	    	   autoState = 12;
 	    		}	
 	   			break;
 	   	     case 12:
	    		_intakeRight.set(0);
	    		_intakeLeft.set(0);
	    		_drive.drive(0, 0);
	        	_drive.setInvertedMotor(MotorType.kFrontLeft, true);
	    	   	_drive.setInvertedMotor(MotorType.kRearLeft, true);
			  break;
 	   }
    	
    }
    
    public void Run_Center_Left_Switch(){
      	NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    	NetworkTableEntry tx = table.getEntry("tx");
    	NetworkTableEntry ty = table.getEntry("ty");
    	NetworkTableEntry ta = table.getEntry("ta");
    	double x = tx.getDouble(0);
    	double y = ty.getDouble(0);
    	double area = ta.getDouble(0);
    	float Kp = -0.1f;
    	double error = x;
    	double steering_adjust = x * Kp;
    	switch(autoState)
 	   {
    	    case 0:
    	    	table.getEntry("ledMode").setNumber(1);
    	    	_drive.setInvertedMotor(MotorType.kFrontLeft, false);
 	   	   		_drive.setInvertedMotor(MotorType.kRearLeft, false);
 	   	   		_drive.drive(-.10, 0);
 	   	   		if(autoCounter >= 27){
 	   	   			autoState = 1;
 	   	   			autoCounter = 0;
 	   	   		}
    	    	break;
 	   		case 1:
 	   		table.getEntry("ledMode").setNumber(0);
 	   			error = 0;
 	   			_drive.setInvertedMotor(MotorType.kFrontLeft, false);
 	   	   		_drive.setInvertedMotor(MotorType.kRearLeft, false);
 	   			_drive.drive(-.5, steering_adjust *-1);//test to see if needed to be a negative
 	   			//_drive.arcadeDrive(0, steering_adjust);
 	   			if(autoCounter >= 2000 || area > 15){
 	   			autoState = 2;
 	   			autoCounter = 0;
 	   			}
 	   			break;
 	   		case 2:
 	   	        /*
 	   	         * _drive.setInvertedMotor(MotorType.kFrontLeft, true);
 	   	   		   _drive.setInvertedMotor(MotorType.kRearLeft, true);
 	   			   _drive.drive(0, 0);		
 	   	         */
 	    		_intakeRight.set(1);
 	    		_intakeLeft.set(1);
 	   			_drive.drive(-.45, 0);
 	   		if(autoCounter >= 30){
 	   			autoState = 3;
 	   			autoCounter = 0;
 	   			}
 	   			break;
 	   		case 3:
 	    		_intakeRight.set(0);
 	    		_intakeLeft.set(0);
 	   			_drive.drive(0, 0);
 	   			if(autoCounter >= 10){
 	   				autoState = 4;
 	   				autoCounter = 0;
 	   			}
 	   		case 4:
 	   			_drive.drive(.30,0);
 	   			if(autoCounter >= 65){
 	   				autoState = 5;
 	   				autoCounter = 0;
 	   			}
 	   			break;
 	   		case 5:
 	   			_drive.drive(0, 0);
 	   			if(autoCounter >2.5){
 	   				autoState = 6;
 	   				autoCounter = 0;
 	   			}
 	   			break;
 	   		case 6:
 	   			while(gyro.getAngle() > -27){
 	   			_drive .drive(.4, 1);
 	   			}
 	   			_drive.drive(0, 0);
 	   			if(autoCounter > 2.5){
 	   				autoState = 7;
 	   				autoCounter = 0;
 	   			}
 	   			break;
 	   		case 7:
 	   			_drive.drive(-.6, 0);
 	   			if(autoCounter > 61){
 	   				autoState = 8;
 	   				autoCounter = 0;
 	   			}
 	   			break;
 	   		case 8:
 	   			_drive.drive(0, 0);
 	   			if(autoCounter > 2.5){
 	   				autoState = 9;
 	   				autoCounter = 0;
 	   			}
 	   			break;
 	   		case 9:
 	   			while(_elevator.getSelectedSensorPosition(0) <= 17550){
 	   				_elevator.set(ControlMode.PercentOutput, -100);
 	   			}
 	   			_elevator.set(0);
 	   			autoState = 10;
 	   			autoCounter = 0;
 	   			break;
 	   		case 10:
 	   			_drive.drive(-.35, 0);
 	   			if(autoCounter > 42){
 	   				_drive.drive(0, 0);
 	   				autoState = 11;
 	   				autoCounter = 0;
 	   			}
 	   				
 	   			break;
 	   		case 11:
 	    		_intakeRight.set(-1);
 	    		_intakeLeft.set(-1);
 	    		_drive.drive(.3, 0);
 	    		if(autoCounter > 15){
 	 	    	   autoCounter = 0;
 	 	    	   autoState = 12;
 	    		}	
 	   			break;
 	   	     case 12:
	    		_intakeRight.set(0);
	    		_intakeLeft.set(0);
	    		_drive.drive(0, 0);
	        	_drive.setInvertedMotor(MotorType.kFrontLeft, true);
	    	   	_drive.setInvertedMotor(MotorType.kRearLeft, true);
			  break;
 	   }
    }
    
    
    public void Run_Right_Right_Switch(){
    	NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    	NetworkTableEntry tx = table.getEntry("tx");
    	NetworkTableEntry ty = table.getEntry("ty");
    	NetworkTableEntry ta = table.getEntry("ta");
    	double x = tx.getDouble(0);
    	double y = ty.getDouble(0);
    	double area = ta.getDouble(0);
    	float Kp = -0.1f;
    	double error = x;
    	double steering_adjust = x * Kp;
    	
    	switch(autoState){
    	case 0:
	    	table.getEntry("ledMode").setNumber(1);
	    	_drive.setInvertedMotor(MotorType.kFrontLeft, false);
	   	   		_drive.setInvertedMotor(MotorType.kRearLeft, false);
	   	   		_drive.drive(-.8, gyro.getAngle() * Kp * 1);
	   	   		if(autoCounter >= 135){
	   	   			autoState = 1;
	   	   			autoCounter = 0;
	   	   		}
	    	break;
	   		case 1:
	   			while(gyro.getAngle() > -115){//-88
	 	   			_drive .drive(.4, 1);
	 	   			}
	 	   			_drive.drive(0, 0);
	 	   			if(autoCounter > 2.5){
	 	   				autoState = 2;
	 	   				autoCounter = 0;
	 	   			}	
	 	   			break;
	   		case 2:
	   			//_drive.drive(-.8, 0);
	 	   			if(autoCounter > 2.5){//23
	 	   				autoCounter = 0;
	 	   				autoState = 3;
	 	   			}
	   			break;
	   		case 3:
	   			/**
	   			while(gyro.getAngle() > -162){
	 	   			_drive .drive(.4, -1);
	 	   			}
	 	   			_drive.drive(0, 0);
	 	   			*/
	 	   			autoState = 4;
	 	   			autoCounter = 0;
	   			break;
	   			
	   			case 4:
	   	 	   		table.getEntry("ledMode").setNumber(2);
	   	 	   			error = 0;
	   	 	   			_drive.setInvertedMotor(MotorType.kFrontLeft, false);
	   	 	   	   		_drive.setInvertedMotor(MotorType.kRearLeft, false);
	   	 	   			_drive.drive(-.20, steering_adjust *-1);//test to see if needed to be a negative
	   	 	   			//_drive.arcadeDrive(0, steering_adjust);
	   	 	   			if(autoCounter >= 2000 || area > 15){
	   	 	   			autoState = 5;
	   	 	   			autoCounter = 0;
	   	 	   			}
	   	 	   			break;
	   	 	   		case 5:
	   	 	   	        /*
	   	 	   	         * _drive.setInvertedMotor(MotorType.kFrontLeft, true);
	   	 	   	   		   _drive.setInvertedMotor(MotorType.kRearLeft, true);
	   	 	   			   _drive.drive(0, 0);		
	   	 	   	         */
	   	 	    		_intakeRight.set(1);
	   	 	    		_intakeLeft.set(1);
	   	 	   			_drive.drive(-.35, 0);
	   	 	   		if(autoCounter >= 50){
	   	 	   			autoState = 6;
	   	 	   			autoCounter = 0;
	   	 	   			}
	   	 	   			break;
	   	 	   		case 6:
	   	 	    		_intakeRight.set(0);
	   	 	    		_intakeLeft.set(0);
	   	 	   			_drive.drive(0, 0);
	   	 	   			if(autoCounter >= 10){
	   	 	   				autoState = 7;
	   	 	   				autoCounter = 0;
	   	 	   			}
	   	 	   		case 7:
	   	 	   			_drive.drive(.30,0);
	   	 	    		_intakeRight.set(1);
	   	 	    		_intakeLeft.set(1);
	   	 	   			if(autoCounter >= 45){
	   	 	   				autoState = 8;
	   	 	   				autoCounter = 0;
	   	 	   			}
	   	 	   			break;
	   	 	   		case 8:
	   	 	    		_intakeRight.set(0);
	   	 	    		_intakeLeft.set(0);
	   	 	   			_drive.drive(0, 0);
	   	 	   			if(autoCounter >2.5){
	   	 	   				autoState = 9;
	   	 	   				autoCounter = 0;
	   	 	   			}
	   	 	   			break;
	   	 	   		case 9:
	   	 	   			/**
	   	 	   		while(gyro.getAngle() < -165){
		 	   			_drive .drive(.4, 1);
		 	   			}
		 	   			_drive.drive(0, 0);
		 	   			*/
		 	   			autoState = 10;
		 	   			autoCounter = 0;
	   	 	   			break;
	   	 	   		case 10:
	   	 	   			while(_elevator.getSelectedSensorPosition(0) <= 17550){//18350
	   	 	   				_elevator.set(ControlMode.PercentOutput, 100);
	   	 	   			}
	   	 	   			_elevator.set(0);
	   	 	   			autoState = 11;
	   	 	   			autoCounter = 0;
	   	 	   			break;
	   	 	   		case 11:
	   	 	   			_drive.drive(-.35, 0);
	   	 	   			if(autoCounter > 57){
	   	 	   				_drive.drive(0, 0);
	   	 	   				autoState = 12;
	   	 	   				autoCounter = 0;
	   	 	   			}
	   	 	   				
	   	 	   			break;
	   	 	   		case 12:
	   	 	    		_intakeRight.set(-1);
	   	 	    		_intakeLeft.set(-1);
	   	 	    		_drive.drive(.3, 0);
	   	 	    		if(autoCounter > 15){
	   	 	 	    	   autoCounter = 0;
	   	 	 	    	   autoState = 13;
	   	 	    		}	
	   	 	   			break;
	   	 	   	     case 13:
	   		    		_intakeRight.set(0);
	   		    		_intakeLeft.set(0);
	   		    		_drive.drive(0, 0);
	   		     	_drive.setInvertedMotor(MotorType.kFrontLeft, true);
	   			   	_drive.setInvertedMotor(MotorType.kRearLeft, true);
	   				  break;
	   	 	   		
    	}
    }
    
    public void Run_Right_Left_Switch(){
    	NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    	NetworkTableEntry tx = table.getEntry("tx");
    	NetworkTableEntry ty = table.getEntry("ty");
    	NetworkTableEntry ta = table.getEntry("ta");
    	double x = tx.getDouble(0);
    	double y = ty.getDouble(0);
    	double area = ta.getDouble(0);
    	float Kp = -0.1f;
    	double error = x;
    	double steering_adjust = x * Kp;
    	
    	switch(autoState){
    	case 0:
	    	table.getEntry("ledMode").setNumber(1);
	    	_drive.setInvertedMotor(MotorType.kFrontLeft, false);
	   	   		_drive.setInvertedMotor(MotorType.kRearLeft, false);
	   	   		_drive.drive(-.8, gyro.getAngle() * Kp * 1);
	   	   		if(autoCounter >= 131.5){//136
	   	   			autoState = 1;
	   	   			autoCounter = 0;
	   	   		}
	    	break;
	   		case 1:
	   			while(gyro.getAngle() > -70){
	 	   			_drive .drive(.4, 1);
	 	   			}
	 	   			_drive.drive(0, 0);
	 	   			if(autoCounter > 2.5){
	 	   				autoState = 2;
	 	   				autoCounter = 0;
	 	   			}	
	 	   			break;
	   		case 2:
	   			_drive.drive(-.5, 0);
	 	   			if(autoCounter > 97.5){//127.5
	 	   				autoCounter = 0;
	 	   				autoState = 3;
	 	   			}
	   			break;
	   		case 3:
	   			while(gyro.getAngle() > -148){//-162
	 	   			_drive .drive(.4, 1);
	 	   			}
	 	   			_drive.drive(0, 0);
	 	   			autoState = 4;
	 	   			autoCounter = 0;
	   			break;
	   			
	   			case 4:
	   	 	   		table.getEntry("ledMode").setNumber(2);
	   	 	   			error = 0;
	   	 	   			_drive.setInvertedMotor(MotorType.kFrontLeft, false);
	   	 	   	   		_drive.setInvertedMotor(MotorType.kRearLeft, false);
	   	 	   			_drive.drive(-.20, steering_adjust *-1);//test to see if needed to be a negative
	   	 	   			//_drive.arcadeDrive(0, steering_adjust);
	   	 	   			if(autoCounter >= 2000 || area > 15){
	   	 	   			autoState = 5;
	   	 	   			autoCounter = 0;
	   	 	   			}
	   	 	   			break;
	   	 	   		case 5:
	   	 	   	        /*
	   	 	   	         * _drive.setInvertedMotor(MotorType.kFrontLeft, true);
	   	 	   	   		   _drive.setInvertedMotor(MotorType.kRearLeft, true);
	   	 	   			   _drive.drive(0, 0);		
	   	 	   	         */
	   	 	    		_intakeRight.set(1);
	   	 	    		_intakeLeft.set(1);
	   	 	   			_drive.drive(-.30, 0);
	   	 	   		if(autoCounter >= 30.5){
	   	 	   			autoState = 6;
	   	 	   			autoCounter = 0;
	   	 	   			}
	   	 	   			break;
	   	 	   		case 6:
	   	 	    		_intakeRight.set(0);
	   	 	    		_intakeLeft.set(0);
	   	 	   			_drive.drive(0, 0);
	   	 	   			if(autoCounter >= 10){
	   	 	   				autoState = 7;
	   	 	   				autoCounter = 0;
	   	 	   			}
	   	 	   		case 7:
	   	 	   			_drive.drive(.2,0);
	   	 	    		_intakeRight.set(1);
	   	 	    		_intakeLeft.set(1);
	   	 	   			if(autoCounter >= 37.5){
	   	 	   				autoState = 8;
	   	 	   				autoCounter = 0;
	   	 	   			}
	   	 	   			break;
	   	 	   		case 8:
	   	 	    		_intakeRight.set(0);
	   	 	    		_intakeLeft.set(0);
	   	 	   			_drive.drive(0, 0);
	   	 	   			if(autoCounter >2.5){
	   	 	   				autoState = 9;
	   	 	   				autoCounter = 0;
	   	 	   			}
	   	 	   			break;
	   	 	   		case 9:
	   	 	   			
	   	 	   		while(gyro.getAngle() < -160){
		 	   			_drive .drive(.4, -1);
		 	   			}
		 	   			_drive.drive(0, 0);
		 	   			
		 	   			autoState = 10;
		 	   			autoCounter = 0;
	   	 	   			break;
	   	 	   		case 10:
	   	 	   			while(_elevator.getSelectedSensorPosition(0) <= 17550){
	   	 	   				_elevator.set(ControlMode.PercentOutput, 100);
	   	 	   			}
	   	 	   			_elevator.set(0);
	   	 	   			autoState = 11;
	   	 	   			autoCounter = 0;
	   	 	   			break;
	   	 	   		case 11:
	   	 	   			_drive.drive(-.35, 0);
	   	 	   			if(autoCounter > 43){
	   	 	   				_drive.drive(0, 0);
	   	 	   				autoState = 12;
	   	 	   				autoCounter = 0;
	   	 	   			}
	   	 	   				
	   	 	   			break;
	   	 	   		case 12:
	   	 	    		_intakeRight.set(-1);
	   	 	    		_intakeLeft.set(-1);
	   	 	    		_drive.drive(.3, 0);
	   	 	    		if(autoCounter > 15){
	   	 	 	    	   autoCounter = 0;
	   	 	 	    	   autoState = 13;
	   	 	    		}	
	   	 	   			break;
	   	 	   	     case 13:
	   		    		_intakeRight.set(0);
	   		    		_intakeLeft.set(0);
	   		    		_drive.drive(0, 0);
	   		     	_drive.setInvertedMotor(MotorType.kFrontLeft, true);
	   			   	_drive.setInvertedMotor(MotorType.kRearLeft, true);
	   				  break;
	   	 	   		
    	}
    }
    
    public void Run_Left_Left_Switch(){
    	NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    	NetworkTableEntry tx = table.getEntry("tx");
    	NetworkTableEntry ty = table.getEntry("ty");
    	NetworkTableEntry ta = table.getEntry("ta");
    	double x = tx.getDouble(0);
    	double y = ty.getDouble(0);
    	double area = ta.getDouble(0);
    	float Kp = -0.1f;
    	double error = x;
    	double steering_adjust = x * Kp;
    	
    	switch(autoState){
    	case 0:
	    	table.getEntry("ledMode").setNumber(1);
	    	_drive.setInvertedMotor(MotorType.kFrontLeft, false);
	   	   		_drive.setInvertedMotor(MotorType.kRearLeft, false);
	   	   		_drive.drive(-.8, gyro.getAngle() * Kp * 1);
	   	   		if(autoCounter >= 139){
	   	   			autoState = 1;
	   	   			autoCounter = 0;
	   	   		}
	    	break;
	   		case 1:
	   			while(gyro.getAngle() < 112){
	 	   			_drive .drive(.4, -1);
	 	   			}
	 	   			_drive.drive(0, 0);
	 	   			if(autoCounter > 2.5){
	 	   				autoState = 2;
	 	   				autoCounter = 0;
	 	   			}	
	 	   			break;
	   		case 2:
	   			
	   			//_drive.drive(-.6, 0);
	 	   			if(autoCounter > 2){//24
	 	   				autoCounter = 0;
	 	   				autoState = 3;
	 	   			}
	   			break;
	   		case 3:
	   			/**
	   			while(gyro.getAngle() < 165.5){
	 	   			_drive .drive(.4, 1);
	 	   			}
	 	   			_drive.drive(0, 0);
	 	   			*/
	 	   			autoState = 4;
	 	   			autoCounter = 0;
	   			break;
	   			
	   			case 4:
	   	 	   		table.getEntry("ledMode").setNumber(2);
	   	 	   			error = 0;
	   	 	   			_drive.setInvertedMotor(MotorType.kFrontLeft, false);
	   	 	   	   		_drive.setInvertedMotor(MotorType.kRearLeft, false);
	   	 	   			_drive.drive(-.20, steering_adjust *-1);//test to see if needed to be a negative
	   	 	   			//_drive.arcadeDrive(0, steering_adjust);
	   	 	   			if(autoCounter >= 2000 || area > 15){
	   	 	   			autoState = 5;
	   	 	   			autoCounter = 0;
	   	 	   			}
	   	 	   			break;
	   	 	   		case 5:
	   	 	   	        /*
	   	 	   	         * _drive.setInvertedMotor(MotorType.kFrontLeft, true);
	   	 	   	   		   _drive.setInvertedMotor(MotorType.kRearLeft, true);
	   	 	   			   _drive.drive(0, 0);		
	   	 	   	         */
	   	 	    		_intakeRight.set(1);
	   	 	    		_intakeLeft.set(1);
	   	 	   			_drive.drive(-.4, 0);
	   	 	   		if(autoCounter >= 50){
	   	 	   			autoState = 6;
	   	 	   			autoCounter = 0;
	   	 	   			}
	   	 	   			break;
	   	 	   		case 6:
	   	 	    		_intakeRight.set(0);
	   	 	    		_intakeLeft.set(0);
	   	 	   			_drive.drive(0, 0);
	   	 	   			if(autoCounter >= 10){
	   	 	   				autoState = 7;
	   	 	   				autoCounter = 0;
	   	 	   			}
	   	 	   		case 7:
	   	 	   			_drive.drive(.30,0);
	   	 	    		_intakeRight.set(1);
	   	 	    		_intakeLeft.set(1);
	   	 	   			if(autoCounter >= 45){
	   	 	   				autoState = 8;
	   	 	   				autoCounter = 0;
	   	 	   			}
	   	 	   			break;
	   	 	   		case 8:
	   	 	    		_intakeRight.set(0);
	   	 	    		_intakeLeft.set(0);
	   	 	   			_drive.drive(0, 0);
	   	 	   			if(autoCounter >2.5){
	   	 	   				autoState = 9;
	   	 	   				autoCounter = 0;
	   	 	   			}
	   	 	   			break;
	   	 	   		case 9:
	   	 	   		/*	
	   	 	   		while(gyro.getAngle() > 165){
		 	   			_drive .drive(.4, -1);
		 	   			}
		 	   			_drive.drive(0, 0);
		 	   			*/
		 	   			autoState = 10;
		 	   			autoCounter = 0;
	   	 	   			break;
	   	 	   		case 10:
	   	 	   			while(_elevator.getSelectedSensorPosition(0) <= 17550){
	   	 	   				_elevator.set(ControlMode.PercentOutput, 100);
	   	 	   			}
	   	 	   			_elevator.set(0);
	   	 	   			autoState = 11;
	   	 	   			autoCounter = 0;
	   	 	   			break;
	   	 	   		case 11:
	   	 	   			_drive.drive(-.35, 0);
	   	 	   			if(autoCounter > 60){
	   	 	   				_drive.drive(0, 0);
	   	 	   				autoState = 12;
	   	 	   				autoCounter = 0;
	   	 	   			}
	   	 	   				
	   	 	   			break;
	   	 	   		case 12:
	   	 	    		_intakeRight.set(-1);
	   	 	    		_intakeLeft.set(-1);
	   	 	    		_drive.drive(.3, 0);
	   	 	    		if(autoCounter > 15){
	   	 	 	    	   autoCounter = 0;
	   	 	 	    	   autoState = 13;
	   	 	    		}	
	   	 	   			break;
	   	 	   	     case 13:
	   		    		_intakeRight.set(0);
	   		    		_intakeLeft.set(0);
	   		    		_drive.drive(0, 0);
	   		     	_drive.setInvertedMotor(MotorType.kFrontLeft, true);
	   			   	_drive.setInvertedMotor(MotorType.kRearLeft, true);
	   				  break;
	   	 	   		
    	}
    }
    
    public void Run_Left_Right_Switch(){
    	NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    	NetworkTableEntry tx = table.getEntry("tx");
    	NetworkTableEntry ty = table.getEntry("ty");
    	NetworkTableEntry ta = table.getEntry("ta");
    	double x = tx.getDouble(0);
    	double y = ty.getDouble(0);
    	double area = ta.getDouble(0);
    	float Kp = -0.1f;
    	double error = x;
    	double steering_adjust = x * Kp;
    	
    	switch(autoState){
    	case 0:
	    	table.getEntry("ledMode").setNumber(1);
	    	_drive.setInvertedMotor(MotorType.kFrontLeft, false);
	   	   		_drive.setInvertedMotor(MotorType.kRearLeft, false);
	   	   		_drive.drive(-.8, gyro.getAngle() * Kp);
	   	   		if(autoCounter >= 130){//123
	   	   			autoState = 1;
	   	   			autoCounter = 0;
	   	   		}
	    	break;
	   		case 1:
	   			while(gyro.getAngle() < 77){
	 	   			_drive .drive(.4, -1);
	 	   			}
	 	   			_drive.drive(0, 0);
	 	   			if(autoCounter > 2.5){
	 	   				autoState = 2;
	 	   				autoCounter = 0;
	 	   			}	
	 	   			break;
	   		case 2:
	   			_drive.drive(-.5, 0);
	 	   			if(autoCounter > 127.75){
	 	   				autoCounter = 0;
	 	   				autoState = 3;
	 	   			}
	   			break;
	   		case 3:
	   			while(gyro.getAngle() < 122){//-162 122
	 	   			_drive .drive(.4, -1);
	 	   			}
	 	   			_drive.drive(0, 0);
	 	   			autoState = 4;
	 	   			autoCounter = 0;
	   			break;
	   			
	   			case 4:
	   	 	   		table.getEntry("ledMode").setNumber(2);
	   	 	   			error = 0;
	   	 	   			_drive.setInvertedMotor(MotorType.kFrontLeft, false);
	   	 	   	   		_drive.setInvertedMotor(MotorType.kRearLeft, false);
	   	 	   			_drive.drive(-.20, steering_adjust *-1);//test to see if needed to be a negative
	   	 	   			//_drive.arcadeDrive(0, steering_adjust);
	   	 	   			if(autoCounter >= 2000 || area > 15){
	   	 	   			autoState = 5;
	   	 	   			autoCounter = 0;
	   	 	   			}
	   	 	   			break;
	   	 	   		case 5:
	   	 	   	        /*
	   	 	   	         * _drive.setInvertedMotor(MotorType.kFrontLeft, true);
	   	 	   	   		   _drive.setInvertedMotor(MotorType.kRearLeft, true);
	   	 	   			   _drive.drive(0, 0);		
	   	 	   	         */
	   	 	    		_intakeRight.set(1);
	   	 	    		_intakeLeft.set(1);
	   	 	   			_drive.drive(-.30, 0);
	   	 	   		if(autoCounter >= 32){
	   	 	   			autoState = 6;
	   	 	   			autoCounter = 0;
	   	 	   			}
	   	 	   			break;
	   	 	   		case 6:
	   	 	    		_intakeRight.set(0);
	   	 	    		_intakeLeft.set(0);
	   	 	   			_drive.drive(0, 0);
	   	 	   			if(autoCounter >= 10){
	   	 	   				autoState = 7;
	   	 	   				autoCounter = 0;
	   	 	   			}
	   	 	   		case 7:
	   	 	   			_drive.drive(.2,0);
	   	 	    		_intakeRight.set(1);
	   	 	    		_intakeLeft.set(1);
	   	 	   			if(autoCounter >= 37.5){
	   	 	   				autoState = 8;
	   	 	   				autoCounter = 0;
	   	 	   			}
	   	 	   			break;
	   	 	   		case 8:
	   	 	    		_intakeRight.set(0);
	   	 	    		_intakeLeft.set(0);
	   	 	   			_drive.drive(0, 0);
	   	 	   			if(autoCounter >2.5){
	   	 	   				autoState = 9;
	   	 	   				autoCounter = 0;
	   	 	   			}
	   	 	   			break;
	   	 	   		case 9:
	   	 	   			
	   	 	   		while(gyro.getAngle() < 158){
		 	   			_drive .drive(.4, -1);
		 	   			}
		 	   			_drive.drive(0, 0);
		 	   			
		 	   			autoState = 10;
		 	   			autoCounter = 0;
	   	 	   			break;
	   	 	   		case 10:
	   	 	   			while(_elevator.getSelectedSensorPosition(0) <= 17550){
	   	 	   				_elevator.set(ControlMode.PercentOutput, 100);
	   	 	   			}
	   	 	   			_elevator.set(0);
	   	 	   			autoState = 11;
	   	 	   			autoCounter = 0;
	   	 	   			break;
	   	 	   		case 11:
	   	 	   			_drive.drive(-.35, 0);
	   	 	   			if(autoCounter > 43){
	   	 	   				_drive.drive(0, 0);
	   	 	   				autoState = 12;
	   	 	   				autoCounter = 0;
	   	 	   			}
	   	 	   				
	   	 	   			break;
	   	 	   		case 12:
	   	 	    		_intakeRight.set(-1);
	   	 	    		_intakeLeft.set(-1);
	   	 	    		_drive.drive(.3, 0);
	   	 	    		if(autoCounter > 15){
	   	 	 	    	   autoCounter = 0;
	   	 	 	    	   autoState = 13;
	   	 	    		}	
	   	 	   			break;
	   	 	   	     case 13:
	   		    		_intakeRight.set(0);
	   		    		_intakeLeft.set(0);
	   		    		_drive.drive(0, 0);
	   		     	_drive.setInvertedMotor(MotorType.kFrontLeft, true);
	   			   	_drive.setInvertedMotor(MotorType.kRearLeft, true);
	   				  break;
	   	 	   		
    	}
    }
    
    
    public void Run_Baseline_Auto(){
    	NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    	NetworkTableEntry tx = table.getEntry("tx");
    	NetworkTableEntry ty = table.getEntry("ty");
    	NetworkTableEntry ta = table.getEntry("ta");
    	double x = tx.getDouble(0);
    	double y = ty.getDouble(0);
    	double area = ta.getDouble(0);
    	float Kp = -0.1f;
    	double error = x;
    	double steering_adjust = x * Kp;
    	
    	switch(autoState){
    	case 0:
    		table.getEntry("snapshot").setNumber(1);
	    	table.getEntry("ledMode").setNumber(1);
	    	_drive.setInvertedMotor(MotorType.kFrontLeft, false);
	   	   		_drive.setInvertedMotor(MotorType.kRearLeft, false);
	   	   		//_drive.drive(-.5, gyro.getAngle() * Kp);
	   	   		_drive.drive(-.5, 0);
	   	   		if(autoCounter >= 140){//123 140
	   	   			autoState = 1;
	   	   			autoCounter = 0;
	   	   		}
	    	break;
	   		case 1:
	   			_drive.setInvertedMotor(MotorType.kFrontLeft, true);
	   	   		_drive.setInvertedMotor(MotorType.kRearLeft, true);
	 	   			_drive.drive(0, 0);
	 	   			if(autoCounter > 2.5){
	 	   				autoState = 2;
	 	   				autoCounter = 0;
	 	   			}	
	 	   			break;
    	}
    }
    
    public void Run_Preload_Auto(){
    	NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    	NetworkTableEntry tx = table.getEntry("tx");
    	NetworkTableEntry ty = table.getEntry("ty");
    	NetworkTableEntry ta = table.getEntry("ta");
    	double x = tx.getDouble(0);
    	double y = ty.getDouble(0);
    	double area = ta.getDouble(0);
    	float Kp = -0.1f;
    	double error = x;
    	double steering_adjust = x * Kp;
    	
    	switch(autoState){
    	case 0:
    		if(autoCounter > 155){
    			autoState = 1;
    			autoCounter = 0;
    		}
    		break;
    	
    	case 1:
	    	table.getEntry("ledMode").setNumber(1);
	    	_drive.setInvertedMotor(MotorType.kFrontLeft, false);
	   	   		_drive.setInvertedMotor(MotorType.kRearLeft, false);
	   	   		_drive.drive(-.5, 0);
	   	   		if(autoCounter >= 200){//123
	   	   			autoState = 2;
	   	   			autoCounter = 0;
	   	   		}
	    	break;
	   		case 2:
	 	   			_drive.drive(-.6, 0);
	 	   			if(autoCounter > 6){
	 	   				autoState = 3;
	 	   				autoCounter = 0;
	 	   			}	
	 	   			break;
	   		case 3:
	   			_drive.drive(0, 0);
	   			_intakeRight.set(1);
	 	    	_intakeLeft.set(1);
	   			if(autoCounter > 15){
	 	   		autoState = 4;
	 	   		autoCounter = 0;
	 	   	    _intakeRight.set(0);
	    		_intakeLeft.set(0);
	   			}
	   			break;
	   		case 4:
	   			while(_elevator.getSelectedSensorPosition(0) <= 17550){
 	   				_elevator.set(ControlMode.PercentOutput, -100);
 	   			}
 	   			_elevator.set(0);
 	   			autoState = 5;
 	   			autoCounter = 0;
	   			break;
	   		case 5:
	   			_drive.drive(0, 0);//.3
	   	    	_drive.setInvertedMotor(MotorType.kFrontLeft, true);
	   		   	_drive.setInvertedMotor(MotorType.kRearLeft, true);
	   			if(autoCounter > 40){
	   				autoState = 5;
	   				_drive.drive(0,0);
	   			}
	   			break;
    	}
    }
    
    public void Run_leftPreload_Right_Auto(){
    	NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    	NetworkTableEntry tx = table.getEntry("tx");
    	NetworkTableEntry ty = table.getEntry("ty");
    	NetworkTableEntry ta = table.getEntry("ta");
    	double x = tx.getDouble(0);
    	double y = ty.getDouble(0);
    	double area = ta.getDouble(0);
    	float Kp = -0.1f;
    	double error = x;
    	double steering_adjust = x * Kp;
    	
    	switch(autoState){
    	case 0:
    		if(autoCounter > 155){
    			autoState = 1;
    			autoCounter = 0;
    		}
    		break;
    	
    	case 1:
    		_drive.setInvertedMotor(MotorType.kFrontLeft, false);
   	   		_drive.setInvertedMotor(MotorType.kRearLeft, false);
   	        _drive.drive(-.5, 0);
   	        if(autoCounter > 30){
   	        	autoState = 2;
   	        	autoCounter = 0;
   	        }
    		break;
    		
    	case 2:
    		while(gyro.getAngle() < 88){
    			_drive.drive(-.35, 1);
    		}
    		_drive.drive(0, 0);
    		_drive.drive(-.5, 0);
    		if(autoCounter > 74){
    			_drive.drive(0, 0);   		
    			autoState = 3;
    			autoCounter = 0;
    		}
    		break;
    		
    	case 3:
    		while(gyro.getAngle() > 0){
    			_drive.drive(-.35, -1);
    		}
    		_drive.drive(0, 0);
    		if(autoCounter > 2.5){
    			autoState = 4;
    			autoCounter = 0;
    		}
    		break;
    		
    	case 4:
	    	table.getEntry("ledMode").setNumber(1);
	    	_drive.setInvertedMotor(MotorType.kFrontLeft, false);
	   	   		_drive.setInvertedMotor(MotorType.kRearLeft, false);
	   	   		_drive.drive(-.5, 0);
	   	   		if(autoCounter >= 200){//123
	   	   			autoState = 5;
	   	   			autoCounter = 0;
	   	   		}
	    	break;
	   		case 5:
	 	   			_drive.drive(-.6, 0);
	 	   			if(autoCounter > 6){
	 	   				autoState = 6;
	 	   				autoCounter = 0;
	 	   			}	
	 	   			break;
	   		case 6:
	   			_drive.drive(0, 0);
	   			_intakeRight.set(-1);
	 	    	_intakeLeft.set(-1);
	   			if(autoCounter > 15){
	 	   		autoState = 7;
	 	   		autoCounter = 0;
	 	   	    _intakeRight.set(0);
	    		_intakeLeft.set(0);
	   			}
	   			break;
	   		case 7:
	   			while(_elevator.getSelectedSensorPosition(0) <= 17550){
 	   				_elevator.set(ControlMode.PercentOutput, -100);
 	   			}
 	   			_elevator.set(0);
 	   			autoState = 8;
 	   			autoCounter = 0;
	   			break;
	   		case 8:
	   			_drive.drive(0, 0);//.3
	   	    	_drive.setInvertedMotor(MotorType.kFrontLeft, true);
	   		   	_drive.setInvertedMotor(MotorType.kRearLeft, true);
	   			if(autoCounter > 40){
	   				autoState = 9;
	   				_drive.drive(0,0);
	   			}
	   			break;
    	}
    }
    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic() {
    	NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        table.getEntry("ledMode").setNumber(0);
    	NetworkTableEntry tx = table.getEntry("tx");
    	NetworkTableEntry ty = table.getEntry("ty");
    	NetworkTableEntry ta = table.getEntry("ta");
    	double x = tx.getDouble(0);
    	double y = ty.getDouble(0);
    	double area = ta.getDouble(0);
    	float Kp = -0.1f;
    	
    	
    	autoCounter++;
    	//SmartDashboard.putNumber("tomohawkCount", _tomohawk.getEncPosition());
    	//SmartDashboard.putNumber("Sensor Range:", ultra.getRangeInches());
    	SmartDashboard.putString("position choice:", (String) position.getSelected());
    	SmartDashboard.putString("Shovel Knight:", Shovel_Knight);
    	SmartDashboard.putNumber("X value error", x * Kp);
    	SmartDashboard.putNumber("Y Value error", y * Kp);
    	SmartDashboard.putNumber("Area", area);
    	SmartDashboard.putNumber("gyroAngle: ", gyro.getAngle());
    	
    	String gameData;
		gameData = DriverStation.getInstance().getGameSpecificMessage();
     
       	switch(autoSelected) {
       	case defaultAuto:
       		//Run_Default_Auto();
       		break;
       	case SwitchAuto:
       		//aiming for a switch auto
       		switch(positionSelected){
       		case leftPosition:
       			if(gameData.length() > 0)
                {
              	  if(gameData.charAt(0) == 'L')
              	  {
      			//Put left auto code here
              		  Shovel_Knight = "Running switch auto for left position, Switch located in left position.";
              		  Run_Left_Left_Switch();
              	  } else {
      			//Put right auto code here
              		Shovel_Knight = "running switch auto for left position, Switch located in right position " ;
              		Run_Left_Right_Switch();
                 	  }
                }
       			break;
       		case rightPosition:
       			if(gameData.length() > 0)
                {
              	  if(gameData.charAt(0) == 'L')
              	  {
      			//Put left auto code here
              		Shovel_Knight = "running switch auto for right position, switch located in left positon" ;
              		Run_Right_Left_Switch();
              	  } else {
      			//Put right auto code here
              		Shovel_Knight = "running switch auto for right position, switch located in right position" ;
              		Run_Right_Right_Switch();
                 	  }
                }
       			break;
       		case centerPosition:
       			if(gameData.length() > 0)
                {
              	  if(gameData.charAt(0) == 'L')
              	  {
      			//Put left auto code here
              		Shovel_Knight = "running switch auto for center position, switch located in left positon" ;
              		Run_Center_Left_Switch();
               	  } else {
      			//Put right auto code here
              		Shovel_Knight = "running switch auto for center positon, switch located in right position" ;
              		Run_Center_Right_Switch();  
               	  }
                }
       			break;
       		}
       		break;
       	case ScaleAuto:
       		//aiming for a scale auto
       		switch(positionSelected){
       		case leftPosition:
       			if(gameData.length() > 0)
                {
              	  if(gameData.charAt(1) == 'L')
              	  {
      			//Put left auto code here
              		Shovel_Knight = "running scale auto for left position, scale located in left position" ;
              	  } else {
      			//Put right auto code here
              		Shovel_Knight = "running scale auto for left position, scale located in right position" ;
                 	  }
                }
       			break;
       		case rightPosition:
       			if(gameData.length() > 0)
                {
              	  if(gameData.charAt(1) == 'L')
              	  {
      			//Put left auto code here
              		Shovel_Knight = "running scale auto for right position, scale located in left position" ;
              	  } else {
      			//Put right auto code here
              		Shovel_Knight = "running scale auto for right position, scale located in right position" ;
                 	  }
                }
       			break;
       		case centerPosition:
       			if(gameData.length() > 0)
                {
              	  if(gameData.charAt(1) == 'L')
              	  {
      			//Put left auto code here
              		Shovel_Knight = "running scale auto for center position, scale located in left position" ;
              	  } else {
      			//Put right auto code here
              		Shovel_Knight = "running scale auto for center position, scale located in right position" ;
                 	  }
                }
       			break;	
       		}
       		break;
       		
       	case BaseAuto:
       		//going for base does not other selection place code here
       		Shovel_Knight = "going to take the safe route! running baseline auto" ;
       		Run_Baseline_Auto();
       		break;
       	//case PreloadAuto:
       		//Run_Preload_Auto();
       		//break;
       	case PreloadAuto:
       		//aiming for a switch auto
       		switch(positionSelected){
       		case leftPosition:
       			if(gameData.length() > 0)
                {
              	  if(gameData.charAt(0) == 'L')
              	  {
      			//Put left auto code here
              		  Shovel_Knight = "Running switch auto for left position, Switch located in left position.";
              		  //Run_Left_Left_Switch();
              		Run_Preload_Auto();
              	  } else {
      			//Put right auto code here
              		Shovel_Knight = "running switch auto for left position, Switch located in right position " ;
              		//Run_Left_Right_Switch();
              		Run_leftPreload_Right_Auto();
                 	  }
                }
       			break;
       		case rightPosition:
       			if(gameData.length() > 0)
                {
              	  if(gameData.charAt(0) == 'L')
              	  {
      			//Put left auto code here
              		Shovel_Knight = "running switch auto for right position, switch located in left positon" ;
              		//Run_Right_Left_Switch();
              	  } else {
      			//Put right auto code here
              		Shovel_Knight = "running switch auto for right position, switch located in right position" ;
              		//Run_Right_Right_Switch();
              		Run_Preload_Auto();
                 	  }
                }
       			break;
       		case centerPosition:
       			if(gameData.length() > 0)
                {
              	  if(gameData.charAt(0) == 'L')
              	  {
      			//Put left auto code here
              		Shovel_Knight = "running switch auto for center position, switch located in left positon" ;
              		//Run_Center_Left_Switch();
               	  } else {
      			//Put right auto code here
              		Shovel_Knight = "running switch auto for center positon, switch located in right position" ;
              		//Run_Center_Right_Switch();  
               	  }
                }
       			break;
       		}
       		break;
       		
       	
    
       	}
    
    }

    /**
     * This function is called periodically during operator control
     */

    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() {
    	/**
    	double left = _joy.getRawAxis(1); // logitech gampad left X, positive is forward
    	double right = _joy2.getRawAxis(1); //logitech gampad right X, positive means turn right
    	_drive.tankDrive(left, -right);//(forward, turn);
    	*/
    	_drive.setInvertedMotor(MotorType.kFrontLeft, true);
	   	_drive.setInvertedMotor(MotorType.kRearLeft, true);
    	NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    	NetworkTableEntry tx = table.getEntry("tx");
    	NetworkTableEntry ty = table.getEntry("ty");
    	NetworkTableEntry ta = table.getEntry("ta");
    	double x = tx.getDouble(0);
    	double y = ty.getDouble(0);
    	double area = ta.getDouble(0);
    	float Kp = -0.1f;
    	table.getEntry("ledMode").setNumber(0);
    	
    	
    	Shovel_Knight = "Teleop has started! Game time:";
    	SmartDashboard.putString("Shovel Knight:", "Teleop has started");
    	SmartDashboard.putNumber("Game Time:" ,(int)DriverStation.getInstance().getMatchTime());
    	//SmartDashboard.putNumber("elevator position:" , _elevator.getSelectedSensorPosition(0)/10000);
    	SmartDashboard.putNumber("elevator position:" , _elevator.getSelectedSensorPosition(0));
    	SmartDashboard.putNumber("Climb Position:", _climb.getSelectedSensorPosition(0));
    	//new Thread(_plotThread).start();
    	SmartDashboard.putNumber("X value error", x * Kp);
    	SmartDashboard.putNumber("Y Value error", y * Kp);
    	SmartDashboard.putNumber("Area", area);
    	SmartDashboard.putNumber("gyroAngle: ", gyro.getAngle());
    	
    	double left = _joy.getRawAxis(1); // logitech gampad left X, positive is forward


       	double right = _joy2.getRawAxis(1); //logitech gampad right X, positive means turn right

        if(_joy.getRawAxis(3) > .4 && _joy2.getRawAxis(3) > .4){
        	_drive.tankDrive(left*.70, -right*.70);//(forward, turn);
        }else if(_joy.getRawAxis(3) < .6 && _joy2.getRawAxis(3) < .6){
        	_drive.tankDrive(-right*.70, left*.70);//(forward, turn);
        }else{
       	_drive.tankDrive(left*.70, -right*.70);//(forward, turn);
        }
    	
    	
    	//double power = _xBoxController.getRawAxis(3) - _xBoxController.getRawAxis(2);
    	//double rotate = _xBoxController.getRawAxis(0);
    	//_drive.arcadeDrive(rotate * .8, -power *.95);//original
    	
    	/**
    	if(_xBoxController.getRawButton(6)){
    		//elevator going down
    		//_elevator.set(1);
    		_elevator.setSelectedSensorPosition(0, 0, 0);
    	}
    	*/
    	
    	if(_partnerController.getRawButton(6) && _elevator.getSelectedSensorPosition(0) <= 17550){
    		//elevator going down
    		//_elevator.set(1);
    		_elevator.set(ControlMode.PercentOutput, -100); /* 100 % output */
    	}else if(_partnerController.getRawButton(5) && _elevator.getSelectedSensorPosition(0) > 1022){
    		//elevator going up 
    		//_elevator.set(-1);
    		_elevator.set(ControlMode.PercentOutput, 100); /* 100 % output */
    	}else{
    		//elevator off
    		_elevator.set(0);
    	}
    	
    	if(_partnerController.getRawAxis(3)> 0){
    		//intake the cube
    		_intakeRight.set(_partnerController.getRawAxis(3));
    		_intakeLeft.set(_partnerController.getRawAxis(3));
    	}else if(_partnerController.getRawAxis(2) > 0){
    		//spit the cube
    		_intakeRight.set(_partnerController.getRawAxis(2) * -1);
    		_intakeLeft.set(_partnerController.getRawAxis(2) * -1);
    	}else{
    		//off
    		_intakeRight.set(0);
    		_intakeLeft.set(0);
    	}
    	
    	if(_partnerController.getRawButton(2)){
    		//intake the cube
    		_climb.set(ControlMode.PercentOutput, 100); /* 100 % output */
      	}else if(_partnerController.getRawButton(4)){
    		//spit the cube
    		_climb.set(ControlMode.PercentOutput, -100); /*inverted 100 % output */
    	}else{
    		//off
    		_climb.set(0);
    	}
    	
    	
    	//autoSelectedButton = (String) button.getSmartDashboardType();
    	//SmartDashboard.putNumber("tomohawkCount", _tomohawk.getEncPosition());
    	//SmartDashboard.putNumber("Sensor Range:", ultra.getRangeInches());
        
    	
    	
    	/**
    	 * this will be used to adjust encoder position at
    	 * any moment to fix the tomohawk at its highest position or
    	 * at its lowest position
    	 */
    	
    	
    }
    	
    	
	/** quick and dirty threaded plotter */
	class PlotThread implements Runnable {
		Robot robot;

		public PlotThread(Robot robot) {
			this.robot = robot;
		}

		public void run() {
			/*
			 * speed up network tables, this is a test project so eat up all of
			 * the network possible for the purpose of this test.
			 */
			// NetworkTable.setUpdateRate(0.010); /* this suggests each time
			// unit is 10ms in the plot */
			while (true) {
				/* yield for a ms or so - this is not meant to be accurate */
				try {
					Thread.sleep(1);
				} catch (Exception e) {
				}
				/* grab the last signal update from our 1ms frame update */
				double velocity = this.robot._elevator.getSelectedSensorVelocity(0);
				SmartDashboard.putNumber("vel", velocity);
			}
		}
    	//LiveWindow.run();
    

    
    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
    
    }
    
	}
}
