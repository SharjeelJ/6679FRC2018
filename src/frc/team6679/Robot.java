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
    private XboxController secondaryController = new XboxController(1);

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

    // Initialize a PIDController object for the drivetrain alongside a double to keep track of the angle to rotate the robot
    private PIDController turnController;
    private double rotateToAngleRate;

    // TODO Tune PID coefficients
    // Initialize the PIDController coefficients
    private static double kP = 0.05;
    private static double kI = 0.00;
    private static double kD = 0.00;
    private static double kF = 0.02;
    private static double kToleranceDegrees = 2.0f;

    // Initialize a string that will specify which side each of the alliance elements are on based on the DriverStation
    private String allianceElementsLocation = "";

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
        leftIntakeMotor.setInverted(true);
        robotDrive.setSafetyEnabled(true);
        robotDrive.setExpiration(0.1);
        robotDrive.setMaxOutput(0.80);

        // Assigns the ultrasonic sensor and enables it to calculate distances
        ultrasonicSensor = new Ultrasonic(0, 1);
        ultrasonicSensor.setEnabled(true);
        ultrasonicSensor.setAutomaticMode(true);

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

        // Assigns the PIDController object its relevant values
        turnController = new PIDController(kP, kI, kD, kF, navX, this::pidWrite);
        turnController.setInputRange(-180.0f, 180.0f);
        turnController.setOutputRange(-0.75, 0.75);
        turnController.setAbsoluteTolerance(kToleranceDegrees);
        turnController.setContinuous(true);

        // Puts 3 buttons onto the LabView Default Dashboard to choose the autonomous routine
        SmartDashboard.putBoolean("DB/Button 1", false);
        SmartDashboard.putBoolean("DB/Button 2", false);
        SmartDashboard.putBoolean("DB/Button 3", false);

        // Puts 3 sliders onto the LabView Default Dashboard to indicate the autonomous stage the routine is currently on
        SmartDashboard.putNumber("DB/Slider 1", 0.0);
        SmartDashboard.putNumber("DB/Slider 2", 0.0);
        SmartDashboard.putNumber("DB/Slider 3", 0.0);

        // Puts 5 strings onto the LabView Default Dashboard to specify the PID coefficients
        SmartDashboard.putString("DB/String 0", String.valueOf(kP));
        SmartDashboard.putString("DB/String 1", String.valueOf(kI));
        SmartDashboard.putString("DB/String 2", String.valueOf(kD));
        SmartDashboard.putString("DB/String 3", String.valueOf(kF));
        SmartDashboard.putString("DB/String 4", String.valueOf(kToleranceDegrees));
    }

    // Function run in an endless loop throughout all modes
    public void robotPeriodic()
    {
        // Updates the values on the LabView Default Dashboard
        SmartDashboard.putString("DB/String 5", String.valueOf(ultrasonicSensor.getRangeInches()));
        SmartDashboard.putString("DB/String 6", String.valueOf(navX.getAngle()));
        SmartDashboard.putString("DB/String 7", String.valueOf(turnController.getSetpoint()));
    }

    // Function run in an endless loop during the disabled mode
    public void disabledPeriodic()
    {
        // Updates the stored PID coefficients based on the 5 strings on the LabView Default Dashboard
        kP = Double.valueOf(SmartDashboard.getString("DB/String 0", String.valueOf(kP)));
        kI = Double.valueOf(SmartDashboard.getString("DB/String 1", String.valueOf(kI)));
        kD = Double.valueOf(SmartDashboard.getString("DB/String 2", String.valueOf(kD)));
        kF = Double.valueOf(SmartDashboard.getString("DB/String 3", String.valueOf(kF)));
        kToleranceDegrees = Double.valueOf(SmartDashboard.getString("DB/String 4", String.valueOf(kToleranceDegrees)));
        turnController.setPID(kP, kI, kD, kF);
        turnController.setAbsoluteTolerance(kToleranceDegrees);
    }


    // Function run once each time the robot enters autonomous mode
    public void autonomousInit()
    {
        // Resets the navX
        navX.reset();

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
        // Moves the robot forward then turns to the appropriate side and moves towards it and places the cube
        if (SmartDashboard.getBoolean("DB/Button 1", false))
        {
            // Parses the alliance elements location string to check if the autonomous mode being run is appropriate and uses it to determine which side the autonomous would be focusing on for the switch
            if (allianceElementsLocation.length() > 0)
            {
                // Code run if the switch is on the left side
                if (allianceElementsLocation.charAt(0) == 'L')
                {
                    // TODO Left switch autonomous code
                    // Moves onto the next stage of the routine
                    if (SmartDashboard.getNumber("DB/Slider 1", 0.0) == 0.0)
                    {
                        SmartDashboard.putNumber("DB/Slider 1", 1.0);
                    }

                    // Moves onto the next stage of the routine
                    else if (SmartDashboard.getNumber("DB/Slider 1", 0.0) == 1.0)
                    {
                        SmartDashboard.putNumber("DB/Slider 1", 2.0);
                    }
                }

                // Code run if the switch is on the right side
                else if (allianceElementsLocation.charAt(0) == 'R')
                {
                    // TODO Right switch autonomous code
                }
            }
        }

        // Moves the robot forward across the line
        else if (SmartDashboard.getBoolean("DB/Button 2", false))
        {
            // TODO Move across line autonomous code
        }

        // Does nothing
        else robotDrive.stopMotor();
    }

    // Function run in an endless loop during the teleop mode
    public void teleopPeriodic()
    {
        // Sets the boolean that toggles the PIDController to rotate the robot to false
        boolean rotateToAngle = false;

        // Left Bumper - Moves the intake motors to take in a cube
        if (primaryController.getBumper(GenericHID.Hand.kLeft) || secondaryController.getBumper(GenericHID.Hand.kLeft))
        {
            leftIntakeMotor.set(1);
            rightIntakeMotor.set(1);
        }

        // Right Bumper - Moves the intake motors to push out a cube
        else if (primaryController.getBumper(GenericHID.Hand.kRight) || secondaryController.getBumper(GenericHID.Hand.kRight))
        {
            leftIntakeMotor.set(-1);
            rightIntakeMotor.set(-1);
        }

        // Stops the intake motors from moving if neither the Left Bumper or the Right Bumper were pressed
        else
        {
            leftIntakeMotor.set(0);
            rightIntakeMotor.set(0);
        }

        // A Button - Resets the navX
        if (primaryController.getAButtonPressed() && primaryController.getAButtonReleased())
        {
            navX.reset();
        }

        // X Button - Rotates the robot by 90 degrees to the left of the origin position
        else if (primaryController.getXButton())
        {
            turnController.setSetpoint(90.0f);
            rotateToAngle = true;
        }

        // Y Button - Rotates the robot to the 180 (origin) position
        else if (primaryController.getYButton())
        {
            turnController.setSetpoint(179.9f);
            rotateToAngle = true;
        }

        // B Button - Rotates the robot by 90 degrees to the right of the origin position
        else if (primaryController.getBButton())
        {
            turnController.setSetpoint(-90.0f);
            rotateToAngle = true;
        }

        // Rotates the robot by the relevant amount based on the PIDController's set point and whether or not a button was pressed to specify a rotation
        double currentRotationRate;
        if (rotateToAngle)
        {
            turnController.enable();
            currentRotationRate = rotateToAngleRate;
        } else
        {
            turnController.disable();
            currentRotationRate = -primaryController.getRawAxis(0);
        }

        // Moves the robot with the rotation rate being influenced by the PIDController
        try
        {
            robotDrive.arcadeDrive(primaryController.getRawAxis(5), currentRotationRate);
        } catch (RuntimeException ex)
        {
            DriverStation.reportError("Error communicating with drive system:  " + ex.getMessage(), true);
        }

        // Passes on the input from the controller's left and right triggers to move the arm vertically and scales its power while prioritizing control to the primary controller
        if ((primaryController.getRawAxis(3) - primaryController.getRawAxis(2)) != 0)
            armMotor.set((primaryController.getRawAxis(3) - primaryController.getRawAxis(2) * 0.25) * 1);
        else if ((secondaryController.getRawAxis(3) - secondaryController.getRawAxis(2)) != 0)
            armMotor.set((secondaryController.getRawAxis(3) - secondaryController.getRawAxis(2)) * 0.5);

        // Waits for the motors to update for 5ms
        Timer.delay(0.005);
    }


    // Function called by the PIDController based on the navX's yaw angle and PID coefficients
    private void pidWrite(double output)
    {
        rotateToAngleRate = output;
    }
}
