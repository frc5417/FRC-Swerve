package frc.robot.subsystems.drivetrain;

import frc.robot.output.commands.drivetrain.TeleopDrive;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.*;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class SwerveWheelController extends SubsystemBase implements SwerveDrivetrainConstants {

    private static SwerveWheelController instance = null;

    private CANSparkMax frontRightDrive = null;
    private CANSparkMax frontLeftDrive = null;
    private CANSparkMax backRightDrive = null;
    private CANSparkMax backLeftDrive = null;
    
    private SwerveWheel frontRight = null;
    private SwerveWheel frontLeft = null;
    private SwerveWheel backRight = null;
    private SwerveWheel backLeft = null;

    private AHRS gyro = null;

    // Get distance between wheels
    private double r = Math.sqrt((L * L) + (W * W));

    private boolean isFieldCentric = true;
    private boolean gyroEnabled = false;
    
    private SwerveWheelController(){

        frontRightDrive = new CANSparkMax(frontRightDriveID, MotorType.kBrushless);
        frontRightDrive.setInverted(true);
        frontRightDrive.setIdleMode(IdleMode.kBrake);
        frontLeftDrive = new CANSparkMax(frontLeftDriveID, MotorType.kBrushless);
        frontLeftDrive.setIdleMode(IdleMode.kBrake);
        backRightDrive = new CANSparkMax(backRightDriveID, MotorType.kBrushless);
        backRightDrive.setInverted(true);
        backRightDrive.setIdleMode(IdleMode.kBrake);
        backLeftDrive = new CANSparkMax(backLeftDriveID, MotorType.kBrushless);
        backLeftDrive.setIdleMode(IdleMode.kBrake);

        frontRight = new SwerveWheel(frontRightDrive, frontRightTurnID, frontRightEncoderID, frontRightEncoderOffset, "Front Right");
        frontLeft = new SwerveWheel(frontLeftDrive, frontLeftTurnID, frontLeftEncoderID, frontLeftEncoderOffset, "Front Left");
        backRight = new SwerveWheel(backRightDrive, backRightTurnID, backRightEncoderID, backRightEncoderOffset, "Back Right");
        backLeft = new SwerveWheel(backLeftDrive, backLeftTurnID, backLeftEncoderID, backLeftEncoderOffset, "Back Left");

        try {
            gyro = new AHRS(SerialPort.Port.kMXP); 
            gyroEnabled = true;
        } catch (RuntimeException ex ) {
            System.out.println("--------------");
            System.out.println("NavX not plugged in");
            System.out.println("--------------");
            gyroEnabled = false;
        }

        frontRight.enable();
        frontLeft.enable();
        backRight.enable();
        backLeft.enable();
    }

    // x1 = strafe, y1 = speed, x2 = rotation 
    // Holonomic drive
    public void drive(double x1, double y1, double x2, double gyroValue) {

        y1 *= -1;

        // Calculate magnitude of joystick
        double magnitude = Math.sqrt((Math.pow(x1, 2)) + (Math.pow(y1,2)));

        if (magnitude >= 0.15) {

            // I got this bit of code from the NavX website
            if (isFieldCentric == true && gyroEnabled == true) {
                // Convert gyro angle to radians
                double gyro = gyroValue * Math.PI / 180;

                double temp = x1 * Math.cos(gyro) + y1 * Math.sin(gyro); 
                y1 = -x1 * Math.sin(gyro) + y1 * Math.cos(gyro); 
                x1 = temp;
            }

            // -------------------------------------
            // Do the swerve wheel math for speed and angles
            // -------------------------------------
            double a = x1 - x2 * (L / r);
            double b = x1 + x2 * (L / r);
            double c = y1 - x2 * (W / r);
            double d = y1 + x2 * (W / r);

            double frontLeftSpeed = Math.sqrt((b * b) + (c * c));
            double frontRightSpeed = Math.sqrt((b * b) + (d * d));
            double backRightSpeed = Math.sqrt((a * a) + (d * d));
            double backLeftSpeed = Math.sqrt((a * a) + (c * c));

            double backRightAngle = Math.atan2(a, d) * 180 / Math.PI;
            double backLeftAngle = Math.atan2(a, c) * 180 / Math.PI;
            double frontRightAngle = Math.atan2(b, d) * 180 / Math.PI;
            double frontLeftAngle = Math.atan2(b, c) * 180 / Math.PI ;
            // -------------------------------------
            

            // -------------------------------------
            // This bit of code normalizes the speed
            // -------------------------------------
            double max = frontLeftSpeed;
            max = Math.max(max, frontRightSpeed);
            max = Math.max(max, backRightSpeed);
            max = Math.max(max, backLeftSpeed);

            if (max > 1) {
                frontLeftSpeed /= max;
                frontRightSpeed /= max;
                backRightSpeed /= max;
                backLeftSpeed /= max;
            }
            // -------------------------------------

            frontRight.setSetpoint(frontRightAngle);
            frontLeft.setSetpoint(frontLeftAngle);
            backRight.setSetpoint(backRightAngle);
            backLeft.setSetpoint(backLeftAngle);

            frontLeft.setSpeed(frontLeftSpeed);
            frontRight.setSpeed(frontRightSpeed);
            backRight.setSpeed(backRightSpeed);
            backLeft.setSpeed(backLeftSpeed);
        } 
        else {
            frontLeft.setSpeed(0);
            frontRight.setSpeed(0);
            backRight.setSpeed(0);
            backLeft.setSpeed(0);
        }
    }

    // Zero the Gryo
    public void resetGyro() {
        gyro.reset();
    }

    // Get the Gyro Angle (-180 to 180)
    public double gyroAngle() {
        return gyro.getYaw();
    }

    // Set the controller to be field oriented drive
    public void setFOD(boolean value) {
        isFieldCentric = value;
    }

    // Get the current FOD mode
    public boolean getFOD() {
        return isFieldCentric;
    }

    /**
     * @return the instance
     */
    public static SwerveWheelController getInstance() {
        if (instance == null) {
            instance = new SwerveWheelController();
            instance.setDefaultCommand(new TeleopDrive());
        }

        return instance;
    }
}

