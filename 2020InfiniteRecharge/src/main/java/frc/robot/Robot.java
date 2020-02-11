/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
//import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.PIDController;
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
	private String m_autoSelected;
	private final SendableChooser<String> m_chooser = new SendableChooser<>();

	/** Hardware, either Talon could be a Victor */

//	TalonSRX _rightMaster = new TalonSRX(1);
//	TalonSRX _leftMaster = new TalonSRX(10);
	//Falcon Drive
	TalonFX _rightMaster = new TalonFX(23);
	TalonFX _rightSlave = new TalonFX(24);
	TalonFX _leftMaster = new TalonFX(21);
	TalonFX _leftSlave = new TalonFX(22);
	TalonSRX _turret = new TalonSRX(5);
	TalonSRX _transfer = new TalonSRX(8);
	TalonSRX _dogbone = new TalonSRX(7);
	TalonSRX _shooterMaster = new TalonSRX(3);
	TalonSRX _shooterSlave = new TalonSRX(4);
	TalonSRX _intake = new TalonSRX(6);
	TalonSRX _winchMaster = new TalonSRX(1);
	TalonSRX _winchSlave = new TalonSRX(2);
	Compressor _compressor = new Compressor(1);
	DoubleSolenoid _deployIntake = new DoubleSolenoid(1,0,1);
	DoubleSolenoid _hanger = new DoubleSolenoid(1,2,3);
	DoubleSolenoid _crawler = new DoubleSolenoid(1,4,5);
	DoubleSolenoid _controlPanel = new DoubleSolenoid(1,6,7);



	Spark Blinkin = new Spark(0);
	AHRS ahrs = new AHRS(SerialPort.Port.kMXP);

	Boolean Aim = false;
	Boolean Shoot = false;
	Boolean lockedOn = false;
	Boolean ManAim = false;
	Boolean AimGO = true;
	Boolean CenterTurret = false;
	Boolean Start = false;
	Boolean teleopInit = true;
	Boolean limitSwitch =false;
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
	double transfer = 0;
	double dogbone = 0;
	double intake = 0;
	double kP= 0.012;
	double kI= 0.003;
	double kD= 0.00002;
	double forward = 0;
	double turn = 0;
	double range = 500;
	double gyro = 0;
	double roll = 0;
	double shotCurrent = 0;
	double gyroF = 0;
	double speedShooter = 0;
	double potStart = 0;

	//Range of motion for turret in either direction
	int distanceTurret;
	int distanceDrive;
	SupplyCurrentLimitConfiguration falcon = new SupplyCurrentLimitConfiguration(true, 60, 60, 0.001);

	

  
  PIDController AimPID= new PIDController(kP, kI, kD);

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
		_rightMaster.setInverted(true);
		_rightSlave.setInverted(true);
		_leftSlave.setInverted(false);
		_dogbone.setInverted(true);
		_turret.setInverted(false);
		_winchSlave.setInverted(true);
		_shooterMaster.setInverted(false);
    	_turret.setSelectedSensorPosition(0);
		_rightMaster.setSelectedSensorPosition(0);
		_shooterMaster.setSelectedSensorPosition(0);
		_dogbone.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.Analog,0,0);
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
	SmartDashboard.putNumber("Roll", roll);
	SmartDashboard.putNumber("Drive", forward);
	SmartDashboard.putData("Auto choices", m_chooser);
	SmartDashboard.putBoolean("AIM", Aim);
	SmartDashboard.putNumber("turret", turretAim);
	SmartDashboard.putNumber("Shooter Speed", speedShooter);
	SmartDashboard.putNumber("Pot", Potentiometer);
	SmartDashboard.putNumber("dogbone", dogbone);
	SmartDashboard.putNumber("transfer", transfer);
	SmartDashboard.putNumber("Shooter Current", shotCurrent);
	m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
	m_chooser.addOption("My Auto", kCustomAuto);
	m_chooser.addOption("Test", kTest);
	m_chooser.addOption("Spin", kSpin);
	gyro = ahrs.getYaw();
	gyroF = ahrs.getFusedHeading();
	roll = ahrs.getRoll();
	distanceTurret = _turret.getSelectedSensorPosition();
	distanceDrive = _rightMaster.getSelectedSensorPosition();
	speedShooter = _shooterMaster.getSelectedSensorVelocity();
	Potentiometer = _dogbone.getSelectedSensorPosition();
	shotCurrent = _shooterMaster.getSupplyCurrent();
	//Green = 0.77
	//Red = 0.61
	//Purple = 0.03

	if(Aim)
	{
		if(tx>-1.5 && tx<1.5 && tv == 1)
		{
			Blinkin.set(0.77);
			lockedOn = true;
		}
		else
		{
			Blinkin.set(0.61);
			lockedOn = false;
		}
		
		NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
	}

	else 
	{
		Blinkin.set(0.03);
		NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
	}

	tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
	tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);

	if( Aim)
	{
		if(tv==1)
		{
			//turretAim =tx;
			turretAim = AimPID.calculate(tx, 0);
		}
		else
		{
			if(distanceTurret>range-100)
			{
				turretAim = 0.3;

			}
			else if(distanceTurret<(range*-1)+100)
			{
				turretAim = -0.3;
			}
			else if (AimGO)
			{
				turretAim = 0.3;
				AimGO = false;
			}
			
		}
				
	}
	else if(ManAim)
	{
		turretAim =_xboxOp.getX(Hand.kLeft)*0.2;
	}
	else if(CenterTurret)
	{
 	 turretAim = AimPID.calculate((-distanceTurret/10), 0);
	}
	else
	{
		turretAim = 0;
	}

	if(distanceTurret<-range && turretAim>0)
	{
		turretAim=0;
	}
	else if (distanceTurret>range && turretAim<0)
	{
		turretAim=0;
	}
	else
	{
		if(turretAim>0.6)
		{
			turretAim=0.6;
		}
		else if(turretAim<-0.6)
		{
			turretAim=-0.6;
		}
	}
	_turret.set(ControlMode.PercentOutput, turretAim);
	
	//Shoot ball 

	_transfer.set(ControlMode.PercentOutput,transfer);
	_dogbone.set(ControlMode.PercentOutput, dogbone);
	_intake.set(ControlMode.PercentOutput, intake);

	if (Shoot) //add lockedOn && Aim && 
	{
		_shooterMaster.set(ControlMode.PercentOutput, 1.0);
	}
	else
	{
		_shooterMaster.set(ControlMode.PercentOutput, 0);

	}

	_winchMaster.set(ControlMode.PercentOutput, winch);




	if(forward<0.3 && forward>-0.3)
	{
		turn = turn/2;
	}
	/* Arcade Drive using PercentOutput along with Arbitrary Feed Forward supplied by turn */
	_leftMaster.set(ControlMode.PercentOutput, forward, DemandType.ArbitraryFeedForward, -turn);
	_rightMaster.set(ControlMode.PercentOutput, forward, DemandType.ArbitraryFeedForward, turn); 
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
		case kCustomAuto:
		  // Put custom auto code here
		  break;
		  
		case kTest:
		  // Put custom auto code here
		  autoReset();
		  while(Start == false)
		  {
			Start = autoStartDrive();			  
		  }
		  while(distanceDrive<2000 || distanceDrive>-2000)
		  {
			_rightMaster.set(ControlMode.PercentOutput, 0.51);
			_leftMaster.set(ControlMode.PercentOutput, 0.51);
		  }
		  autoReset();

		  break;

		case kDefaultAuto:
		default:
		  // Put default auto code here
		  autoReset();
		  break;

		case kSpin: 
		autoReset();
		while(Start == false)
		{
			Start = autoStartDrive();	
		}
		while(gyro<85)
		{
			_rightMaster.set(ControlMode.PercentOutput, -0.51);
			_leftMaster.set(ControlMode.PercentOutput, 0.51);	
		}
		autoReset();
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
    
		forward = -1 *_xboxDriver.getY(Hand.kLeft);
		turn = -_xboxDriver.getX(Hand.kRight);		
		forward = Deadband(forward);
		turn = Deadband(turn);
		Shoot = _xboxOp.getBumper(Hand.kRight);
		//Aim = _xboxOp.getBumper(Hand.kRight);
		ManAim = _xboxOp.getBumper(Hand.kLeft);
		//AimGO = _xboxOp.getBumperPressed(Hand.kRight);
		//CenterTurret = _xboxOp.getYButton();
		transfer = _xboxOp.getY(Hand.kLeft)*-0.3;
		dogbone =  _xboxOp.getY(Hand.kRight)*0.3;
		if(_xboxOp.getAButton())
		{
			dogbone = 0.5;
			intake = 0.33;
			if(Potentiometer<(potStart+5)) //pot
			{
				transfer = 0.33;
			}
			else
			{
				transfer = 0;
			}
		}
		else
		{
			//dogbone = 0;
			intake = 0;
			//transfer = 0;
		}

		if(_xboxOp.getBButton())
		{
			_dogbone.overrideLimitSwitchesEnable(false);
		}
		else
		{
			_dogbone.overrideLimitSwitchesEnable(true);
		}

		if(_xboxOp.getYButton())
		{
			winch = 0.5;
		}
		else
		{
			winch = 0;
		}
 
  }
  
  double Deadband(final double value) 
	{
		/* Upper deadband */
		if (value >= 0.1 || value <= -0.1) 
			return value;

		/* Outside deadband */
		return 0;
	}
  boolean autoStartDrive()
  {
		while(distanceDrive<100 && distanceDrive>-100)
		{
		  _rightMaster.set(ControlMode.PercentOutput, 0.3);
		  _leftMaster.set(ControlMode.PercentOutput, 0.3);
		}

		  return true;
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
	transfer = 0;
	dogbone = 0;
	intake = 0;
	forward = 0;
	turn = 0;
	//.set(ControlMode.PercentOutput, 0);
	teleopInit = true;

  }

}
