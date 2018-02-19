package frc.team6679;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;

public class Robot extends IterativeRobot
{
    // Initialize the two Xbox 360 controllers to control the robot
    private XboxController driveController = new XboxController(0);
    //    private XboxController functionController = new XboxController(1);

    // Initialize the drivetrain motors
    private WPI_TalonSRX leftDriveMotor1;
    private WPI_TalonSRX leftDriveMotor2;
    private WPI_TalonSRX leftDriveMotor3;
    private WPI_TalonSRX rightDriveMotor1;
    private WPI_TalonSRX rightDriveMotor2;
    private WPI_TalonSRX rightDriveMotor3;

    // Initialize the arm motor
    private VictorSP armMotor;

    // Initialize a PIDController object for the drivetrain alongside a double to keep track of the angle to rotate the robot
    PIDController turnController;
    // Initialize the arm intake motors
    private VictorSP leftIntakeMotor;

    // Pairs up the drivetrain motors based on their respective side and initializes the drivetrain controlling object
    private SpeedControllerGroup leftSideDriveMotors;
    private SpeedControllerGroup rightSideDriveMotors;
    private DifferentialDrive robotDrive;

    // Initialize the navX object
    private AHRS navX;
    private VictorSP rightIntakeMotor;
    double rotateToAngleRate;

    // Initialize the PIDController coefficients
    static final double kP = 0.03;
    static final double kI = 0.00;
    static final double kD = 0.00;
    static final double kF = 0.00;
    static final double kToleranceDegrees = 2.0f;

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

        // Adds the PIDController to the DriverStation's Test panel
        Sendable pidSendable = turnController;
        pidSendable.setName("RotateController");
        pidSendable.setSubsystem("DriveSystem");
        LiveWindow.add(pidSendable);
    }

    // Function run in an endless loop during the teleop mode
    public void teleopPeriodic()
    {
        // Sets the boolean that toggles the PIDController to rotate the robot to false
        boolean rotateToAngle = false;

        // Start Button - Moves the intake motors to take in a cube
        if (driveController.getStartButtonPressed())
        {
            leftIntakeMotor.set(1);
            rightIntakeMotor.set(1);
        }

        // Back Button - Moves the intake motors to push out a cube
        else if (driveController.getBackButtonPressed())
        {
            leftIntakeMotor.set(-1);
            rightIntakeMotor.set(-1);
        }

        // Stops the intake motors from moving if neither the Start Button or the Back Button were pressed
        else
        {
            leftIntakeMotor.set(0);
            rightIntakeMotor.set(0);
        }

        // A Button - Resets the navX
        //        if (driveController.getAButtonPressed() && driveController.getAButtonReleased())
        if (driveController.getAButton())
        {
            navX.reset();
        }

        // X Button - Rotates the robot by 90 degrees to the left
        //        else if (driveController.getXButtonPressed() && driveController.getXButtonReleased())
        else if (driveController.getXButton())
        {
            turnController.setSetpoint(90.0f);
            rotateToAngle = true;
        }

        // Y Button - Rotates the robot by 180 degrees to the right
        //        else if (driveController.getYButtonPressed() && driveController.getYButtonReleased())
        else if (driveController.getYButton())
        {
            turnController.setSetpoint(179.9f);
            rotateToAngle = true;
        }

        // B Button - Rotates the robot by 90 degrees to the right
        //        else if (driveController.getBButtonPressed() && driveController.getBButtonReleased())
        else if (driveController.getBButton())
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
            currentRotationRate = -driveController.getRawAxis(0);
        }

        // Moves the robot with the rotation rate being influenced by the PIDController
        try
        {
            robotDrive.arcadeDrive(driveController.getRawAxis(5), currentRotationRate);
            System.out.println(leftDriveMotor1.getOutputCurrent() + "\t" + leftDriveMotor2.getOutputCurrent() + "\t" + leftDriveMotor3.getOutputCurrent() + "\t|\t" + rightDriveMotor1.getOutputCurrent() + "\t" + rightDriveMotor2.getOutputCurrent() + "\t" + rightDriveMotor3.getOutputCurrent());
        } catch (RuntimeException ex)
        {
            DriverStation.reportError("Error communicating with drive system:  " + ex.getMessage(), true);
        }

        // Passes on the input from the driving controller's left stick's Y axis (acceleration) and right stick's X axis (turning) to move the robot around
        //        robotDrive.arcadeDrive(driveController.getRawAxis(5), -driveController.getRawAxis(0));

        // Passes on the input from the primary controller's left and right triggers to move the arm vertically and scales its power
        armMotor.set((driveController.getRawAxis(3) - driveController.getRawAxis(2)) * 0.5);

        // Waits for the motors to update for 5ms
        Timer.delay(0.005);
    }


    // Function called by the PIDController based on the navX's yaw angle and PID coefficients
    public void pidWrite(double output)
    {
        rotateToAngleRate = output;
    }
}
