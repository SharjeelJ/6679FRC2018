package frc.team6679;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

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

    // Initialize the navX object
    private AHRS navX;

    // Initialize a PIDController object for the drivetrain alongside a double to keep track of the angle to rotate the robot
    private PIDController turnController;
    private double rotateToAngleRate;

    // Initialize the PIDController coefficients
    private static final double kP = 0.05;
    private static final double kI = 0.00;
    private static final double kD = 0.00;
    private static final double kF = 0.02;
    private static final double kToleranceDegrees = 2.0f;

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

        // Attempts to setup the navX object otherwise prints an error
        try
        {
            // Initializes the navX object on the roboRIO's MXP port
            navX = new AHRS(SPI.Port.kMXP);
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

        // X Button - Rotates the robot by 90 degrees to the left
        else if (primaryController.getXButton())
        {
            turnController.setSetpoint(90.0f);
            rotateToAngle = true;
        }

        // Y Button - Rotates the robot by 180 degrees to the right
        else if (primaryController.getYButton())
        {
            turnController.setSetpoint(179.9f);
            rotateToAngle = true;
        }

        // B Button - Rotates the robot by 90 degrees to the right
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

        // Passes on the input from the controller's left and right triggers to move the arm vertically and scales its power and prioritizes control to the primary controller
        if ((primaryController.getRawAxis(3) - primaryController.getRawAxis(2)) != 0)
            armMotor.set((primaryController.getRawAxis(3) - primaryController.getRawAxis(2)) * 0.5);
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
