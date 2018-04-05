package frc.team6679;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends IterativeRobot
{
    // Initialize the two Xbox 360 controllers to control the robot
    private XboxController primaryController = new XboxController(0);

    // Initialize the drivetrain motors
    private WPI_TalonSRX leftDriveMotor1;
    private WPI_TalonSRX leftDriveMotor2;
    private WPI_TalonSRX leftDriveMotor3;
    private WPI_TalonSRX rightDriveMotor1;
    private WPI_TalonSRX rightDriveMotor2;
    private WPI_TalonSRX rightDriveMotor3;

    // Initialize the arm motor
    private VictorSP armMotor;

    // Initialize the arm intake motors
    private VictorSP leftIntakeMotor;
    private VictorSP rightIntakeMotor;

    // Pairs up the drivetrain motors based on their respective side and initializes the drivetrain controlling object
    private SpeedControllerGroup leftSideDriveMotors;
    private SpeedControllerGroup rightSideDriveMotors;
    private DifferentialDrive robotDrive;

    // Initialize an ultrasonic sensor
    private Ultrasonic ultrasonicSensor;

    // Initialize the navX object
    private AHRS navX;

    // Initialize a string that will specify which side each of the alliance elements are on based on the DriverStation
    private String allianceElementsLocation = "";

    // Initialize a pushbutton sensor for determining when to disable the arm motor's break mode
    private DigitalInput armPushButton;

    // Initialize a boolean to enable a slow break mode for the arm motor
    private boolean armBreakMode = false;

    // Initialize a timer for timing routines
    private Timer timer = new Timer();

    // Function run once when the robot is turned on
    public void robotInit()
    {
        // Assigns all the motors to their respective objects (the number in brackets is the port # of what is connected where)
        leftDriveMotor1 = new WPI_TalonSRX(0);
        leftDriveMotor2 = new WPI_TalonSRX(1);
        leftDriveMotor3 = new WPI_TalonSRX(2);
        rightDriveMotor1 = new WPI_TalonSRX(3);
        rightDriveMotor2 = new WPI_TalonSRX(4);
        rightDriveMotor3 = new WPI_TalonSRX(5);
        armMotor = new VictorSP(0);
        leftIntakeMotor = new VictorSP(1);
        rightIntakeMotor = new VictorSP(2);

        // Assigns the drivetrain motors to their respective motor controller group and then passes them on to the drivetrain controller object
        leftSideDriveMotors = new SpeedControllerGroup(leftDriveMotor1, leftDriveMotor2, leftDriveMotor3);
        rightSideDriveMotors = new SpeedControllerGroup(rightDriveMotor1, rightDriveMotor2, rightDriveMotor3);
        robotDrive = new DifferentialDrive(leftSideDriveMotors, rightSideDriveMotors);

        // Sets the appropriate configuration settings for the motors
        leftSideDriveMotors.setInverted(true);
        rightSideDriveMotors.setInverted(true);
        robotDrive.setSafetyEnabled(true);
        robotDrive.setExpiration(0.1);
        robotDrive.setMaxOutput(0.80);

        // Assigns the ultrasonic sensor and enables it to calculate distances
        ultrasonicSensor = new Ultrasonic(0, 1);
        ultrasonicSensor.setEnabled(true);
        ultrasonicSensor.setAutomaticMode(true);

        // Assigns the arm's pushbutton sensor
        armPushButton = new DigitalInput(8);

        // Attempts to setup the navX object otherwise prints an error
        try
        {
            // Initializes the navX object on the roboRIO's MXP port and resets it
            navX = new AHRS(SPI.Port.kMXP);
            navX.reset();
        } catch (RuntimeException ex)
        {
            DriverStation.reportError("Error instantiating navX-MXP:  " + ex.getMessage(), true);
        }

        // Puts 3 buttons onto the LabView Default Dashboard to choose the autonomous routine
        SmartDashboard.putBoolean("DB/Button 1", false);
        SmartDashboard.putBoolean("DB/Button 2", false);
        SmartDashboard.putBoolean("DB/Button 3", false);

        // Puts 3 sliders onto the LabView Default Dashboard to indicate the autonomous stage the routine is currently on
        SmartDashboard.putNumber("DB/Slider 1", 0.0);
        SmartDashboard.putNumber("DB/Slider 2", 0.0);
        SmartDashboard.putNumber("DB/Slider 3", 0.0);
    }

    // Function run in an endless loop throughout all modes
    public void robotPeriodic()
    {
        // Updates the values on the LabView Default Dashboard
        SmartDashboard.putString("DB/String 5", String.valueOf(ultrasonicSensor.getRangeInches()));
        SmartDashboard.putString("DB/String 6", String.valueOf(navX.getAngle()));
    }

    // Function run once each time the robot enters autonomous mode
    public void autonomousInit()
    {
        // Resets the navX
        navX.reset();

        // Resets the timer
        timer.reset();

        // Resets the 3 autonomous stage sliders
        SmartDashboard.putNumber("DB/Slider 1", 0.0);
        SmartDashboard.putNumber("DB/Slider 2", 0.0);
        SmartDashboard.putNumber("DB/Slider 3", 0.0);

        // Gets the string from the DriverStation specifying which side each of the alliance elements are on and stores it
        allianceElementsLocation = DriverStation.getInstance().getGameSpecificMessage();
    }

    // Function run in an endless loop during the autonomous mode
    public void autonomousPeriodic()
    {
        // Checks to see which autonomous routine has been requested for and calls it based on the LabView Default Dashboard's buttons
        // Moves the robot forward then places the cube if it is on the left side
        if (SmartDashboard.getBoolean("DB/Button 1", false))
        {
            // Parses the alliance elements location string to check if the autonomous mode being run is appropriate and uses it to determine which side the autonomous would be focusing on for the switch
            if (allianceElementsLocation.length() > 0)
            {
                // Moves the robot forward towards the switch
                if (SmartDashboard.getNumber("DB/Slider 1", 0.0) == 0.0)
                {
                    SmartDashboard.putNumber("DB/Slider 1", 1.0);
                    robotDrive.arcadeDrive(0.60, 0);
                }
                // Stops the robot when its up against the switch
                else if (SmartDashboard.getNumber("DB/Slider 1", 0.0) == 1.0 && ultrasonicSensor.getRangeInches() < 15)
                {
                    SmartDashboard.putNumber("DB/Slider 1", 2.0);
                    robotDrive.stopMotor();
                }

                // Code run if the switch is on the left side
                if (allianceElementsLocation.charAt(0) == 'L')
                {
                    // Places the cube
                    if (SmartDashboard.getNumber("DB/Slider 1", 0.0) == 2.0)
                    {
                        SmartDashboard.putNumber("DB/Slider 1", 3.0);
                        armMotor.set(0.75);
                        timer.reset();
                        timer.start();
                    }
                    // Stops the arm motor
                    else if (SmartDashboard.getNumber("DB/Slider 1", 0.0) == 3.0 && timer.get() > 2)
                    {
                        SmartDashboard.putNumber("DB/Slider 1", 4.0);
                        armMotor.stopMotor();
                        timer.stop();
                        timer.reset();
                    }
                }
            }
        }
        // Moves the robot forward then places the cube if it is on the right side
        if (SmartDashboard.getBoolean("DB/Button 2", false))
        {
            // Parses the alliance elements location string to check if the autonomous mode being run is appropriate and uses it to determine which side the autonomous would be focusing on for the switch
            if (allianceElementsLocation.length() > 0)
            {
                // Moves the robot forward towards the switch
                if (SmartDashboard.getNumber("DB/Slider 2", 0.0) == 0.0)
                {
                    SmartDashboard.putNumber("DB/Slider 2", 1.0);
                    robotDrive.arcadeDrive(0.60, 0);
                }
                // Stops the robot when its up against the switch
                else if (SmartDashboard.getNumber("DB/Slider 2", 0.0) == 1.0 && ultrasonicSensor.getRangeInches() < 15)
                {
                    SmartDashboard.putNumber("DB/Slider 2", 2.0);
                    robotDrive.stopMotor();
                }

                // Code run if the switch is on the right side
                if (allianceElementsLocation.charAt(0) == 'R')
                {
                    // Places the cube
                    if (SmartDashboard.getNumber("DB/Slider 2", 0.0) == 2.0)
                    {
                        SmartDashboard.putNumber("DB/Slider 2", 3.0);
                        armMotor.set(0.75);
                        timer.reset();
                        timer.start();
                    }
                    // Stops the arm motor
                    else if (SmartDashboard.getNumber("DB/Slider 2", 0.0) == 3.0 && timer.get() > 2)
                    {
                        SmartDashboard.putNumber("DB/Slider 2", 4.0);
                        armMotor.stopMotor();
                        timer.stop();
                        timer.reset();
                    }
                }
            }
        }
        // Moves the robot forward across the line
        else if (SmartDashboard.getBoolean("DB/Button 3", false))
        {
            if (SmartDashboard.getNumber("DB/Slider 3", 0.0) == 0.0)
            {
                SmartDashboard.putNumber("DB/Slider 3", 1.0);
                robotDrive.arcadeDrive(-0.60, 0);
            }
            // Stops the robot when it has crossed the line
            else if (SmartDashboard.getNumber("DB/Slider 3", 0.0) == 1.0 && ultrasonicSensor.getRangeInches() > 180)
            {
                SmartDashboard.putNumber("DB/Slider 3", 2.0);
                robotDrive.stopMotor();
            }
        }
        // Does nothing
        else
        {
            robotDrive.stopMotor();
            armMotor.stopMotor();
        }
    }

    // Function run in an endless loop during the teleop mode
    public void teleopPeriodic()
    {
        // Left Bumper - Moves the intake motors to push out a cube
        if (primaryController.getBumper(GenericHID.Hand.kLeft))
        {
            leftIntakeMotor.set(-1);
            rightIntakeMotor.set(-1);
        }
        // Right Bumper - Moves the intake motors to take in a cube
        else if (primaryController.getBumper(GenericHID.Hand.kRight))
        {
            leftIntakeMotor.set(1);
            rightIntakeMotor.set(1);
        }
        // Stops the intake motors from moving if neither the Left Bumper or the Right Bumper were pressed
        else
        {
            leftIntakeMotor.set(0);
            rightIntakeMotor.set(0);
        }

        // Passes on the input from the primary controller's left and right triggers to move the arm vertically and scales its power
        if (primaryController.getTriggerAxis(GenericHID.Hand.kRight) >= 0.2)
        {
            armMotor.set(primaryController.getTriggerAxis(GenericHID.Hand.kRight));
            armBreakMode = true;
        }
        // Lowers the arm by counteracting a consistent upward power forcing a slow drop rate
        else if (primaryController.getTriggerAxis(GenericHID.Hand.kLeft) >= 0.2)
        {
            armMotor.set((-primaryController.getTriggerAxis(GenericHID.Hand.kLeft) * 0.35) + 0.15);
        }
        // Makes sure that the arm does not slam to the bottom by enabling a consistent upwards power to slow it down
        else if (armBreakMode)
        {
            // Checks to see if the arm's pushbutton sensor is not being triggered and stops the break mode if it is
            if (!armPushButton.get())
            {
                armBreakMode = false;
                armMotor.set(0);
            } else armMotor.set(0.15);
        }
        // Stops the arm motor
        else armMotor.set(0);

        // Waits for the motors to update for 5ms
        Timer.delay(0.005);
    }
}
