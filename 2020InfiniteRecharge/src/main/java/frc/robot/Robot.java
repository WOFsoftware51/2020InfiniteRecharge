/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;


import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
//import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot 
{
	private static final String kDefaultAuto = "Default";
	private static final String kCustomAuto = "My Auto";
	private static final String kTest = "Test";
	private static final String kSpin = "Spin";
	private static final String kAutoCenter = "Center Shot";
	private static final String kAutoLeft = "HOT Shot";
	private static final String kAutoTrench = "Trench Shot";
	private String m_autoSelected;
	private final SendableChooser<String> m_chooser = new SendableChooser<>();
	private final SendableChooser<Integer> s_chooser = new SendableChooser<>();

	/** Hardware, either Talon could be a Victor */
	//TalonSRX _rightMaster = new TalonSRX(1);
	//TalonSRX _leftMaster = new TalonSRX(10);
	//Falcon Drive

	WPI_TalonFX _rightMaster = new WPI_TalonFX(23);
	WPI_TalonFX _rightSlave = new WPI_TalonFX(24);
	WPI_TalonFX _leftMaster = new WPI_TalonFX(21);
	WPI_TalonFX _leftSlave = new WPI_TalonFX(22);

	private final ColorMatch m_colorMatcher = new ColorMatch();

	TalonSRX _turret = new TalonSRX(5);
	TalonSRX _transfer = new TalonSRX(8);
	TalonSRX _dogbone = new TalonSRX(7);
	TalonSRX _shooterMaster = new TalonSRX(3);
	TalonSRX _shooterSlave = new TalonSRX(4);
	TalonSRX _intake = new TalonSRX(6);
	TalonSRX _winchMaster = new TalonSRX(1);
	TalonSRX _winchSlave = new TalonSRX(2);
	TalonSRX _traverser = new TalonSRX(9);
	TalonSRX _wheelOfFortune = new TalonSRX(10);

	Compressor _compressor = new Compressor(1);

	DoubleSolenoid _deployIntake = new DoubleSolenoid(1,0,7);
	DoubleSolenoid _winchBrake = new DoubleSolenoid(2,2,5);
	DoubleSolenoid _crawler = new DoubleSolenoid(2,1,6);
	DoubleSolenoid _hangPull = new DoubleSolenoid(1,2,5);
	DoubleSolenoid _FlapperL = new DoubleSolenoid(2,3,4);
	DoubleSolenoid _FlapperR = new DoubleSolenoid(1,1,6);

	Servo _rightRelease = new Servo(1);
	Servo _leftRelease = new Servo(2);

	Spark Blinkin = new Spark(0);
	AHRS ahrs = new AHRS(SerialPort.Port.kMXP);
	DigitalInput proxSensor = new DigitalInput(0);

	Boolean Aim = false;
	Boolean Shoot = false;
	Boolean lockedOn = false;
	Boolean ManAim = false;
	Boolean AimGO = false;
	Boolean AimGo2 = false;
	Boolean Hanging = false;
	Boolean CenterTurret = false;
	Boolean Start = false;
	Boolean teleopInit = true;
	Boolean limitSwitch =false;
	Boolean winchBrake = false;
	Boolean deployIntake = false;
	Boolean crawler = false;
	Boolean hangPull = false;
	Boolean FlapperL = false;
	Boolean FlapperR = false;
	Boolean Pulse = false;
	Boolean humanLoad = false;
	Boolean eat = false;
	Boolean SlowBot = false;
	Boolean AutoShot2 = false;
	Boolean Fire = false;
	Boolean Spin = false;
	Boolean ballSensor = false;

	//Joystick _gamepad = new Joystick(1);
	XboxController _xboxDriver = new XboxController(0);
	XboxController _xboxOp = new XboxController(1);

	double tv = 0;
	double Potentiometer = 0;

	//tv : Whether the limelight has any valid targets (0 or 1)
	double tx = 0;

	//tx : Horizontal Offset From Crosshair To Target (LL1: -27 degrees to 27 degrees | LL2: -29.8 to 29.8 degrees)
	double turretAim= 0;
	double winch = 0;
	int count = 0;
	double transfer = 0;
	double dogbone = 0;
	double traverser = 0;
	double intake = 0;

	final double turretP= 0.051;
	final double turretI= 0.0;//0.051;
	final double turretD= 0.0015;//0.002;
	final double shootP= .2;
	final double shootI= 0.0;
	final double shootD= .02;
	final double shootF= 0.006;//6;

	double forward = 0;
	double turn = 0;
	double range = 3000;
	double gyro = 0;
	double roll = 0;
	double shotCurrent = 0;
	double kSpeed = 7255*8192/600;
	double gyroF = 0;
	double speedShooter = 0;
	double potStart = 0;
	int speedAdd = 0;
	int Timer = 0;
	int autonState = 0;
	int wheel = 0;
	String gameData;


	private final static int DRIVE_BACKWARDS = 1;
 	private final static int STOP = 2;
	private final static int SHOOT = 3;
	private final static int FINISH = 4;
	private final static int DRIVE_BACKWARDS2 = 5;
	private final static int DRIVE_FORWARD = 6;
	private final static int START = 0;

	//Range of motion for turret in either direction
	int distanceTurret;
	int distanceDrive;
	SupplyCurrentLimitConfiguration falcon = new SupplyCurrentLimitConfiguration(true, 60, 60, 0.001);
	DifferentialDrive Cdrive = new DifferentialDrive(_leftMaster, _rightMaster);
	String colString;
	///neverrest 1120

  
  	PIDController AimPID= new PIDController(turretP, turretI, turretD);

  	/**
  	 * This function is run when the robot is first started up and should be
  	 * used for any initialization code.
  	 */

  	@Override
  	public void robotInit() 
 	 {
		/* Ensure motor output is neutral during init */
		_leftMaster.set(ControlMode.PercentOutput, 0);
		_rightMaster.set(ControlMode.PercentOutput, 0);
		_rightMaster.configSupplyCurrentLimit(falcon);
		_leftMaster.configSupplyCurrentLimit(falcon);
		_rightSlave.configSupplyCurrentLimit(falcon);
		_leftSlave.configSupplyCurrentLimit(falcon);
		_turret.configContinuousCurrentLimit(10);
		_turret.configPeakCurrentLimit(29);
		_dogbone.configContinuousCurrentLimit(10);
		_transfer.configContinuousCurrentLimit(10);
		_shooterMaster.set(ControlMode.PercentOutput, 0);
		_turret.set(ControlMode.PercentOutput, 0);
		_dogbone.set(ControlMode.PercentOutput, 0);
		_transfer.set(ControlMode.PercentOutput, 0);
		_winchMaster.set(ControlMode.PercentOutput, 0);
		_intake.set(ControlMode.PercentOutput, 0);
		_leftSlave.follow(_leftMaster);
		_rightSlave.follow(_rightMaster);
		_winchSlave.follow(_winchMaster);
		_shooterSlave.follow(_shooterMaster);
		_shooterMaster.config_kF(0, shootF, 30);
		_shooterMaster.config_kP(0, shootP, 30);
		_shooterMaster.config_kI(0, shootI, 30);
		_shooterMaster.config_kD(0, shootD, 30);
		_shooterMaster.setSensorPhase(true);
		_compressor.start();
		
		/* Factory Default all hardware to prevent unexpected behaviour */

		/* Set Neutral mode */
		_shooterSlave.setNeutralMode(NeutralMode.Coast);
		_shooterMaster.setNeutralMode(NeutralMode.Coast);
		_turret.setNeutralMode(NeutralMode.Coast);
		_winchMaster.setNeutralMode(NeutralMode.Brake);
		_winchSlave.setNeutralMode(NeutralMode.Brake);

		/* Configure output direction */
		_leftMaster.setInverted(false);
		_rightMaster.setInverted(false);
		_rightSlave.setInverted(false);
		_leftSlave.setInverted(false);
		_dogbone.setInverted(true);
		_turret.setInverted(true);
		_intake.setInverted(true);
		_winchSlave.setInverted(true);
		_shooterMaster.setInverted(false);
		_shooterSlave.setInverted(false);
    	_turret.setSelectedSensorPosition(0);
		_rightMaster.setSelectedSensorPosition(0);
		_shooterMaster.setSelectedSensorPosition(0);
		_wheelOfFortune.setSelectedSensorPosition(0);
		_dogbone.setSelectedSensorPosition(0);
		ahrs.zeroYaw();
		potStart = _dogbone.getSelectedSensorPosition();
 	 }

 	 /**
	  * This function is called every robot packet, no matter the mode. Use
  	  * this for items like diagnostics that you want ran during disabled,
	  * autonomous, teleoperated and test.
  	  *
   	  * <p>This runs after the mode specific periodic functions, but before
  	  * LiveWindow and SmartDashboard integrated updating.
   	  */
  @Override

  	public void robotPeriodic() 
 	{
   	 SmartDashboard.putNumber("distanceTurret", distanceTurret);
   	 SmartDashboard.putNumber("Gyro", gyro);
  	 SmartDashboard.putNumber("distanceDrive", distanceDrive);
	 SmartDashboard.putNumber("Gyro Fused", gyroF);
	 SmartDashboard.putNumber("Drive", forward);
	 SmartDashboard.putData("Auto choices", m_chooser);
	 SmartDashboard.putData("Shot Speed", s_chooser);
	 SmartDashboard.putBoolean("AIM", Aim);
	 SmartDashboard.putBoolean("spin", Spin);
	 SmartDashboard.putBoolean("fire", Fire);
	 SmartDashboard.putBoolean("eat", eat);
	 SmartDashboard.putBoolean("shoot", Shoot);
	 SmartDashboard.putBoolean("humanload", humanLoad);
	 SmartDashboard.putNumber("turret", turretAim);
	 SmartDashboard.putNumber("Shooter Speed", speedShooter/8192*600);  //8192 ticks per rev, 600 counts per minute
	 SmartDashboard.putNumber("Shooter Speed Raw ", speedShooter);  //8192 ticks per rev, 600 counts per minute
	 SmartDashboard.putNumber("Shooter Speed Desired", kSpeed);  //8192 ticks per rev, 600 counts per minute
	 SmartDashboard.putNumber("Pot", Potentiometer);
	 SmartDashboard.putNumber("Forward", forward);
	 SmartDashboard.putNumber("turn", turn);  //8192 ticks per rev, 600 counts per minute
	 SmartDashboard.putBoolean("Pulse", Pulse);
	 SmartDashboard.putNumber("dogbone", dogbone);
	 SmartDashboard.putNumber("count", count);
	 SmartDashboard.putNumber("transfer", transfer);
	 SmartDashboard.putNumber("Shooter Current", shotCurrent);
	// SmartDashboard.putString("Color", colString);
	SmartDashboard.putNumber("Wheel Distance", wheel);
	SmartDashboard.putBoolean("Ball Proximity", ballSensor);
	 m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
	 m_chooser.addOption("My Auto", kCustomAuto);
	 m_chooser.addOption("Test", kTest);
	 m_chooser.addOption("Spin", kSpin);
	 m_chooser.addOption("Hot Shot", kAutoLeft);
	 m_chooser.addOption("Center Shot", kAutoCenter);
	 m_chooser.addOption("Trench Shot", kAutoTrench);
	 s_chooser.setDefaultOption("7255", 0);
	 s_chooser.addOption("7755", 500);
	 s_chooser.addOption("8255", 1000);
	 s_chooser.addOption("8755", 1500);
	 s_chooser.addOption("9255", 2000);

	 gyro = ahrs.getYaw();
	 gyroF = ahrs.getFusedHeading();
	 roll = ahrs.getRoll();

	 distanceTurret = _turret.getSelectedSensorPosition();
	 distanceDrive = _rightMaster.getSelectedSensorPosition();
	 speedShooter = _shooterMaster.getSelectedSensorVelocity();
	 Potentiometer = _dogbone.getSelectedSensorPosition();
	 shotCurrent = _shooterMaster.getSupplyCurrent();
	 wheel = _wheelOfFortune.getSelectedSensorPosition();
	 speedAdd = s_chooser.getSelected();
	 kSpeed = (7255+speedAdd)*8192/600;
	 ballSensor = proxSensor.get();


	 gameData = DriverStation.getInstance().getGameSpecificMessage();

    //Code for no data received yet
		//these are solenoids
		if (winchBrake)
		{
		_winchBrake.set(Value.kForward);
		}
		else
		{
		_winchBrake.set(Value.kReverse);
		}	

		if (deployIntake)
		{
		_deployIntake.set(Value.kForward);
		}
		else
		{
		_deployIntake.set(Value.kReverse);
		}

		if (crawler)
		{
		_crawler.set(Value.kForward);
		}
		else
		{ 	
		_crawler.set(Value.kReverse);
		}

		if (hangPull)
		{
		_hangPull.set(Value.kReverse);
		}
		else
		{ 	
		_hangPull.set(Value.kForward);
		}	

		if (FlapperL)
		{
		_FlapperL.set(Value.kForward);
		}
		else
		{ 	
		_FlapperL.set(Value.kReverse);
		}


		if (FlapperR)
		{
		_FlapperR.set(Value.kForward);
		}
		else
		{ 	
		_FlapperR.set(Value.kReverse);
		}
		
		//Green = 0.77
		//Red = 0.61
		//Purple = 0.03

		if(hangPull)
		{
			Hanging=true;
		}
		if(Aim)
		{
			if(tx>-1.5 && tx<1.5 && tv == 1)
			{
				Blinkin.set(0.75);
				lockedOn = true;
			}
			else
			{
				Blinkin.set(-0.93);
				lockedOn = false;
			}
		
			NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
			NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(0);
		}

		else 
		{
			if(gameData.length() > 0)
 			{
				 switch (gameData.charAt(0))
				 {
					case 'B' :
					Blinkin.set(-0.09);
        			break;
      				case 'G' :
      				Blinkin.set(0.35);
        			break;
      				case 'R' :
      				Blinkin.set(-0.11);
        			break;
      				case 'Y' :
      				Blinkin.set(-0.07); 
        			break;
      				default :
        			//This is corrupt data
        			break;
    			}
			  }
			  else
			  {
				Blinkin.set(0.09);
			  }
	
			NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
			NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(1);
		}

		tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
		tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);

		if(Aim)
		{
			Shoot = true;
			Hanging = false;
			if(tv==1)
			{
				//turretAim =tx;
				turretAim = AimPID.calculate(tx, 0);
			}
			else
			{
				if(distanceTurret>range-700)
				{
					turretAim = -0.4;
				}	

				else if(distanceTurret<(range*-1)+700)
				{
					turretAim = 0.4;
				}
				else if (AimGO)
				{
					turretAim = 0.4;
					AimGO = false;
				}
			
			}
				
		}

		else if(ManAim)
		{
			turretAim =_xboxOp.getX(Hand.kLeft)*-0.3;
			Hanging = false;
		}
		else if(CenterTurret)
		{
			if (distanceTurret<-400)
			{
				turretAim= 0.5;
			}
			else if (distanceTurret>400)
			{
				turretAim= -0.5;
			}
			else if (distanceTurret<-150)
			{
				turretAim= 0.2;
			}
			else if (distanceTurret>150)
			{
				turretAim= -0.2;
			}
			else
			{
				turretAim= 0;
			}
			//turretAim = AimPID.calculate((-distanceTurret/10), 0);
			//bit
			//distanceTurret

			//200
		}
		else
		{
			turretAim = 0;
		}

		
		if(distanceTurret<-range && turretAim<0)
		{
			turretAim=0;
		}
		else if (distanceTurret>range && turretAim>0)
		{
			turretAim=0;
		}
		else
		{
			if(turretAim>0.4)
			{
				turretAim=0.4;
			}
			else if(turretAim<-0.4)
			{
				turretAim=-0.4;
			}

		}
		_turret.set(ControlMode.PercentOutput, turretAim);
		//Shoot ball 
		if (Fire && lockedOn && (speedShooter>(7000*8192/600)))
		{
			_xboxOp.setRumble(RumbleType.kLeftRumble, 1.0);
			_xboxOp.setRumble(RumbleType.kRightRumble, 1.0);
			Shoot = true;
			humanLoad = true;
		}
		else
		{
			_xboxOp.setRumble(RumbleType.kLeftRumble, 0);
			_xboxOp.setRumble(RumbleType.kRightRumble, 0);
		}
		if(humanLoad)
		{
			eat = true;
		}
		if(eat)
		{
			if(humanLoad)
			{
				intake = 0;				
			}
			else
			{
				intake = 0.8;	
			}
		
			if(Potentiometer<50) //pot
			{
				if(Shoot)
				{
					transfer = 0.8;
				}
				else
				{
					transfer = 0.35;
				}
			}
			else if(Shoot)
			{
				transfer = 0.4;
			}
			else
			{
				transfer = 0;
			}
			if(ballSensor && count < 1)
			{
				Pulse = true;
				count++;
				if(Shoot==false)
				{
					dogbone = -0.1;
				}
			}
			else if(Shoot)
			{
				dogbone = 1;
			}
			else if(ballSensor)
			{
				dogbone = 0;
			}
			else
			{
				count=0;
				dogbone = 0.5;
			}
		}
	else
	{
		//dogbone = 0;
		intake = 0;
		//transfer = 0;
	}

	_transfer.set(ControlMode.PercentOutput,transfer);
	_dogbone.set(ControlMode.PercentOutput, dogbone);
	_intake.set(ControlMode.PercentOutput, intake);
	_traverser.set(ControlMode.PercentOutput, traverser);

	if (Shoot) //add lockedOn && Aim && 
	{
		_dogbone.overrideLimitSwitchesEnable(false);
		_shooterMaster.set(ControlMode.Velocity, kSpeed);
		//_shooterMaster.set(ControlMode.PercentOutput, 0.85);
	}
	else
	{
		_dogbone.overrideLimitSwitchesEnable(true);
		_shooterMaster.set(ControlMode.PercentOutput, 0);

	}

	_winchMaster.set(ControlMode.PercentOutput, winch);




	if(forward<0.2 && forward>-0.2)
	{
		SlowBot=true;
		//turn = turn/2;
	}
	else
	{
		SlowBot=false;
	}
	/* Arcade Drive using PercentOutput along with Arbitrary Feed Forward supplied by turn */
	//_leftMaster.set(ControlMode.PercentOutput, forward, DemandType.ArbitraryFeedForward, -turn);
	//_rightMaster.set(ControlMode.PercentOutput, forward, DemandType.ArbitraryFeedForward, turn);
	
	Cdrive.curvatureDrive(forward, -turn*0.4, SlowBot);
}
  

	
  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to
   * the switch structure below with additional strings. If using the
   * SendableChooser make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() 
  {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
	System.out.println("Auto selected: " + m_autoSelected);

	_leftMaster.setNeutralMode(NeutralMode.Brake);
	_rightMaster.setNeutralMode(NeutralMode.Brake);
	_leftSlave.setNeutralMode(NeutralMode.Brake);
	_rightSlave.setNeutralMode(NeutralMode.Brake);
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic()
  {
	switch (m_autoSelected) 
	{
		
		case kDefaultAuto:
			default:
		  	// Put default auto code here
		  	autoReset();
		  	break;
		case kCustomAuto:
		  	// Put custom auto code here
		  	break;
		  
		case kTest:
			Blinkin.set(0.13);
		  	// Put custom auto code here
		  	autoReset();

		  	if(distanceDrive<200000)
		  	{
				forward = -0.2;
				turn = gyro/10;
		  	}
		  	else
		  	{
				autoReset();
		  	}
		  	break;

		case kSpin: 
			Blinkin.set(0.13);
			autoReset();

			if(gyro>-85)
			{
				turn = -0.65;	
			}
			else
			{
		  		autoReset();
			}
			break;

		case kAutoCenter:
			switch (autonState) 
			{
    		case START: 
			{
				autoReset();
				_rightMaster.setSelectedSensorPosition(0);
				_leftMaster.setSelectedSensorPosition(0);
				autonState = DRIVE_BACKWARDS;
				break;
			}
	
			case SHOOT: 
			{
				forward = 0;
				Aim = true;
				Shoot = true;
				FlapperL=true;
				FlapperR=true;				
				if (lockedOn)
				{
					//shoot
					if(Timer>60)
					{
						humanLoad=true;
					}
					Timer++;
				}

				if (Timer >= 160)
				{
					Aim = false;
					if(AutoShot2) 
					{
						autonState = FINISH;

					}
					{
						autonState = DRIVE_BACKWARDS2;
					}
				}	
				break;
			}
	
			case DRIVE_BACKWARDS: 
			{
				forward = -0.4;
				if (distanceDrive >5000)
				{
					AimGO = true;
					autonState = SHOOT;
				}
				break;
			}
			case DRIVE_BACKWARDS2: 
			{
				forward = -0.4;
				if (distanceDrive >15000)
				{
					autonState = STOP;
				}
				break;
			}
			
			case STOP: 
			{
				autoReset();
				break;
			}

			case FINISH: 
			{
				autoReset();
				break;
			}
		}
		case kAutoLeft:
			switch (autonState) 
			{
			case START: 
			{
				autoReset();
				_rightMaster.setSelectedSensorPosition(0);
				_leftMaster.setSelectedSensorPosition(0);
				autonState = DRIVE_BACKWARDS;
				break;
			}

			case SHOOT: 
			{
				forward = 0;
				Aim = true;
				Shoot = true;
				FlapperL=true;
				FlapperR=true;				
				if (lockedOn)
				{
					//shoot
					if(Timer>60)
					{
						humanLoad=true;
					}
				Timer++;
				}

				if (Timer >= 160)
				{
					Aim = false; 
					if(AutoShot2) 
					{
						autonState = FINISH;
					}
					else
					{
						_dogbone.setSelectedSensorPosition(0);
						autonState = DRIVE_BACKWARDS2;
					}
				}	
				break;
			}
			case DRIVE_BACKWARDS:
			{
			forward = -0.4;
			if (distanceDrive >5000)
			{
				AimGO = true;
				autonState = SHOOT;
			}
			break;
		}
		case DRIVE_BACKWARDS2: 
		{
			Shoot = false;
			humanLoad = false;
			eat=true;
			forward = -0.20;
			if (distanceDrive >55000)
			{
				autonState = STOP;
			}
			break;
		}
		
		case STOP: 
		{
			autoReset();
			autonState = DRIVE_FORWARD;
			AimGO = true;
			break;
		}

		case DRIVE_FORWARD: 
		{
			forward = 0.55;
			Aim = true;
			Shoot = true;
			if (distanceDrive < 60000)
			{
				Timer = 40;
				AutoShot2 = true;
				autonState = SHOOT;
			}
			eat=false;
			break;
		}

		case FINISH: 
		{
			autoReset();
			break;
		}
	}
		
		
		
		case kAutoTrench:
			switch (autonState) 
			{
			case START: 
			{
				autoReset();
				_rightMaster.setSelectedSensorPosition(0);
				_leftMaster.setSelectedSensorPosition(0);
				autonState = DRIVE_BACKWARDS;
				break;
			}

			case SHOOT: 
			{
				forward = 0;
				Aim = true;
				Shoot = true;
				FlapperL=true;
				FlapperR=false;				
				if (lockedOn)
				{
					//shoot
					if(Timer>60)
					{
						humanLoad=true;
					}
					if(Timer>100)
					{
						FlapperL=false;
						FlapperR=true;	
					}

				Timer++;
				}

				if (Timer >= 160)
				{
					Aim = false; 
					if(AutoShot2) 
					{
						autonState = FINISH;
					}
					else
					{
						_dogbone.setSelectedSensorPosition(0);
						autonState = DRIVE_BACKWARDS2;
					}
				}	
				break;
			}
			case DRIVE_BACKWARDS:
			{
			forward = -0.4;
			if (distanceDrive >5000)
			{
				AimGO = true;
				autonState = SHOOT;
			}
			break;
		}
		case DRIVE_BACKWARDS2: 
		{
			Shoot = false;
			humanLoad = false;
			eat=true;
			forward = -0.20;
			if (distanceDrive >155000)
			{
				autonState = STOP;
			}
			break;
		}
		
		case STOP: 
		{
			autoReset();
			autonState = DRIVE_FORWARD;
			AimGO = true;
			break;
		}

		case DRIVE_FORWARD: 
		{
			forward = 0.55;
			Aim = true;
			Shoot = true;
			if (distanceDrive < 60000)
			{
				Timer = 40;
				AutoShot2 = true;
				autonState = SHOOT;
			}
			eat=false;
			break;
		}

		case FINISH: 
		{
			autoReset();
			break;
		}
	}
	break;
}
  }
  /**
   * This function is called periodically during operator control.
   */
  
  @Override
  public void testPeriodic() 
  {  
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void teleopPeriodic() 
  {
    
    /**

     * Run the color match algorithm on our detected color

     */
		/* Gamepad processing */
		//double forward = -1 * _gamepad.getY();
		   //double turn = -_gamepad.getTwist();	
		if(teleopInit)
		{
			autoReset();
			_leftMaster.setNeutralMode(NeutralMode.Coast);
			_rightMaster.setNeutralMode(NeutralMode.Coast);
			_leftSlave.setNeutralMode(NeutralMode.Coast);
			_rightSlave.setNeutralMode(NeutralMode.Coast);
			teleopInit = false;
		}
	
		//Solenoid Test Buttons
		forward = -1 *_xboxDriver.getY(Hand.kLeft);
		turn = -_xboxDriver.getX(Hand.kRight);
		forward = Deadband(forward);		
		turn = Deadband(turn);
		turn = turn*turn*turn;
		turn = turn *0.8;
		forward = forward*forward*forward;
		winchBrake = _xboxDriver.getStartButton();
		deployIntake = _xboxDriver.getAButton();
		crawler = _xboxDriver.getYButton();
		hangPull = _xboxOp.getStartButton();

		if(300>_xboxDriver.getPOV()&&_xboxDriver.getPOV()>200)
		{
			FlapperL=true;
		}
		else
		{
			FlapperL=false;
		}

		if(135>_xboxDriver.getPOV()&&_xboxDriver.getPOV()>60)
		{
			FlapperR=true;
		}
		else if(Shoot&&(FlapperL==false))
		{		
			FlapperR=true;
		}
		else
		{
			FlapperR=false;
		}

		if(_xboxDriver.getBumper(Hand.kLeft))
		{
			traverser = 0.6;
		}
		else if(_xboxDriver.getBumper(Hand.kRight))
		{
			traverser = -.6;
		}
		else
		{
			traverser = 0;

		}
		if(_xboxDriver.getTriggerAxis(Hand.kLeft)>0.8)
		{
			_leftMaster.setNeutralMode(NeutralMode.Brake);
			_rightMaster.setNeutralMode(NeutralMode.Brake);
			_leftSlave.setNeutralMode(NeutralMode.Brake);
			_rightSlave.setNeutralMode(NeutralMode.Brake);
		}
		else
		{
			_leftMaster.setNeutralMode(NeutralMode.Coast);
			_rightMaster.setNeutralMode(NeutralMode.Coast);
			_leftSlave.setNeutralMode(NeutralMode.Coast);
			_rightSlave.setNeutralMode(NeutralMode.Coast);

		}

		if(_xboxDriver.getTriggerAxis(Hand.kRight)<0.8)
		{
			forward = forward *0.6;
			_xboxOp.setRumble(RumbleType.kLeftRumble, 0);
			_xboxOp.setRumble(RumbleType.kRightRumble, 0);
		}
		else
		{
			_xboxOp.setRumble(RumbleType.kLeftRumble, 1.0);
			_xboxOp.setRumble(RumbleType.kRightRumble, 1.0);
		}

		if(_xboxOp.getBumper(Hand.kRight))
		{
			Shoot = true;
		}
		else
		{
			if(Aim)
			{
				Shoot = true;
			}
			else
			{
				Shoot = false;
			}
		}

		if(_xboxOp.getTriggerAxis(Hand.kLeft)>0.8)
		{
			Aim = true;
			if(AimGo2)
			{
				AimGO = true;
				AimGo2= false;
			}

		}
		else
		{
			Aim = false;
			AimGO = false;
			AimGo2= true;


		}

		if(_xboxOp.getTriggerAxis(Hand.kRight)<0.8)
		{
			Fire = false;
		}
		else
		{
			Fire = true;
		}
	
		ManAim = _xboxOp.getBumper(Hand.kLeft);
		//CenterTurret = _xboxOp.getYButton();
		transfer = _xboxOp.getY(Hand.kLeft)*-0.3;
		dogbone =  _xboxOp.getY(Hand.kRight)*0.3;

		if(_xboxOp.getXButton())
		{
			Spin = true;
		}
		if(Spin)
		{
			if (wheel > -3300)
			{
				_wheelOfFortune.set(ControlMode.PercentOutput, 0.65);
			}
			else 
			{
				Spin = false;
			}
		}
		else
		{
			_wheelOfFortune.setSelectedSensorPosition(0);
			_wheelOfFortune.set(ControlMode.PercentOutput, 0.0);	
		}
		
		if(_xboxOp.getBackButton())
		{
			_dogbone.setSelectedSensorPosition(0);
		}

		if(270>_xboxOp.getPOV()&&_xboxOp.getPOV()>90)
		{
			CenterTurret = true;
		}
		else 
		{		
			if (Hanging)
			{
				CenterTurret = true;
			}
			else
			{
				CenterTurret = false;
			}
		}

		humanLoad =_xboxOp.getBButton();
		eat = _xboxOp.getAButton();


		if(_xboxOp.getYButton())
		{
			//winchBrake = true; 
			winch = 0.70;
			
		}
		else
		{
			//winchBrake = true;
			winch = 0;
		}
 
  }
  
  double Deadband(final double value) 
	{
		/* Upper deadband */
		if (value >= 0.01 || value <= -0.01) 
			return value;

		/* Outside deadband */
		return 0;
	}
  void autoReset()
  {
	_rightMaster.set(ControlMode.PercentOutput, 0);
	_leftMaster.set(ControlMode.PercentOutput, 0);
	_shooterMaster.set(ControlMode.PercentOutput, 0);
	_turret.set(ControlMode.PercentOutput, 0);
	_dogbone.set(ControlMode.PercentOutput, 0);
	_transfer.set(ControlMode.PercentOutput, 0);
	_winchMaster.set(ControlMode.PercentOutput, 0);
	_intake.set(ControlMode.PercentOutput, 0);
	turretAim= 0;
	winch = 0;
	deployIntake = false;
	transfer = 0;
	dogbone = 0;
	intake = 0;
	forward = 0;
	turn = 0;
	AutoShot2 = false;
	humanLoad = false;
	eat = false;
	Aim = false;


	
	Shoot= false;
	FlapperL=false;
	FlapperR=false;	
	Hanging=false;
	//.set(ControlMode.PercentOutput, 0);
	teleopInit = true;

  }

}
