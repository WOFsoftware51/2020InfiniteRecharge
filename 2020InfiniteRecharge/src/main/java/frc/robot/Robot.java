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
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.Joystick;
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
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
	// Color Sensor
	Spark spark = new Spark(0);
	AHRS ahrs = new AHRS(SerialPort.Port.kMXP);

	/** Hardware, either Talon could be a Victor */
	TalonSRX _turret = new TalonSRX(3);
	TalonSRX _rightMaster = new TalonSRX(1);
//	TalonSRX _rightSlave = new TalonSRX(6);
	TalonSRX _leftMaster = new TalonSRX(10);
//	TalonFX _LeftMaster = new TalonFX(5);
	//AHRS ahrs;
	

	Boolean Aim = false;
	Boolean ManAim = false;
  Boolean AimGO = true;
  Boolean CenterTurret = true;
	Joystick _gamepad = new Joystick(1);
	XboxController _xbox = new XboxController(0);
	double tv = 0;
	//tv : Whether the limelight has any valid targets (0 or 1)
	double tx = 0;
	//tx : Horizontal Offset From Crosshair To Target (LL1: -27 degrees to 27 degrees | LL2: -29.8 to 29.8 degrees)
	double turretAim= 0;
	double kP= 0.012;
	double kI= 0.003;
	double kD= 0.00002;
	double forward = 0;
	double turn = 0;
	double range = 500;
	double gyro = 0;
	double gyroF = 0;
	//Range of motion for turret in either direction
  int distanceTurret;
  int distanceDrive;

  
  PIDController AimPID= new PIDController(kP, kI, kD);

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() 
  {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

		/* Ensure motor output is neutral during init */
		_leftMaster.set(ControlMode.PercentOutput, 0);
		_rightMaster.set(ControlMode.PercentOutput, 0);
		_turret.set(ControlMode.PercentOutput, 0);
		//_leftSlave.follow(_leftMaster);
		//_rightSlave.follow(_rightMaster);

		/* Factory Default all hardware to prevent unexpected behaviour */
		_leftMaster.configFactoryDefault();
		_rightMaster.configFactoryDefault();
		_turret.configFactoryDefault();
		
		/* Set Neutral mode */
		_leftMaster.setNeutralMode(NeutralMode.Coast);
		_rightMaster.setNeutralMode(NeutralMode.Coast);
		_turret.setNeutralMode(NeutralMode.Coast);
		//_leftSlave.setNeutralMode(NeutralMode.Coast);
		//_rightSlave.setNeutralMode(NeutralMode.Coast);
		
		/* Configure output direction */
		_leftMaster.setInverted(false);
		_rightMaster.setInverted(true);
		_turret.setInverted(false);
    _turret.setSelectedSensorPosition(0);
    _rightMaster.setSelectedSensorPosition(0);
		//ahrs = new AHRS(SerialPort.Port.kMXP); /* Alternatives:  SPI.Port.kMXP, I2C.Port.kMXP or SerialPort.Port.kUSB */
		ahrs.zeroYaw();
		

		
		System.out.println("This is Arcade Drive using Arbitrary Feed Forward.");
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
  public void robotPeriodic() {
    SmartDashboard.putNumber("distanceTurret", distanceTurret);
    SmartDashboard.putNumber("Gyro", gyro);
  	SmartDashboard.putNumber("distanceDrive", distanceDrive);
		SmartDashboard.putNumber("Gyro Fused", gyroF);
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
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);

    System.out.println("Auto selected: " + m_autoSelected);
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
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
    
		forward = -1 *_xbox.getY(Hand.kLeft);
		turn = -_xbox.getX(Hand.kRight);		
		forward = Deadband(forward);
		turn = Deadband(turn);
		Aim = _xbox.getBumper(Hand.kRight);
		ManAim = _xbox.getBumper(Hand.kLeft);
		AimGO = _xbox.getBumperPressed(Hand.kRight);
		gyro = ahrs.getYaw();
    gyroF = ahrs.getFusedHeading();
    CenterTurret = _xbox.getYButton();

    distanceTurret = _turret.getSelectedSensorPosition();
    distanceDrive = _rightMaster.getSelectedSensorPosition();

		_turret.getSelectedSensorVelocity();

		if(Aim)
		{
			if(tx>-1.5 && tx<1.5 && tv == 1)
			{
				spark.set(0.77);
			}
			else
			{
				spark.set(0.61);
			}
			
			NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
		}

		else 
		{
			spark.set(0.03);
			NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
		}

		tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
		tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);

		SmartDashboard.putBoolean("AIM", Aim);
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
			turretAim =_xbox.getX(Hand.kLeft)*0.2;
    }
    else if(CenterTurret)
    {
      turretAim = AimPID.calculate((-distanceTurret/10), 0);
    }
		else
		{
			turretAim = 0;
		}

		SmartDashboard.putNumber("turretRAW", turretAim);


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
		SmartDashboard.putNumber("turret", turretAim);



		if(forward<0.3 && forward>-0.3)
		{
			turn = turn/2;
		}

		/* Arcade Drive using PercentOutput along with Arbitrary Feed Forward supplied by turn */
		_leftMaster.set(ControlMode.PercentOutput, forward, DemandType.ArbitraryFeedForward, +turn);
    _rightMaster.set(ControlMode.PercentOutput, forward, DemandType.ArbitraryFeedForward, -turn); 
  }
  double Deadband(final double value) 
	{
		/* Upper deadband */
		if (value >= 0.1 || value <= -0.1) 
			return value;

		/* Outside deadband */
		return 0;
	}
}
