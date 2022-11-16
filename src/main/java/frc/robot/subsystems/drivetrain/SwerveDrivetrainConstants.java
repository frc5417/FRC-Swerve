package frc.robot.subsystems.drivetrain;

public interface SwerveDrivetrainConstants {

	// encoders fixed
	// need to do PID
	// test CAN
	// verify gyro

	// Length and Width of robot in inches
	public final double L = 25;
	public final double W = 27;

	// PIDF Variables
	public final double kP = 0.02;
	public final double kI = 0.0;
	public final double kD = 0.0;
	public final double kF = 0.0;
	
	// Quadrature Encoder Ticks per Rotation
	public final int QUAD_COUNTS_PER_ROT = 42;

	// Talon SRX Turn Motor CAN ID
	public final int frontLeftTurnID = 10;
	public final int frontRightTurnID = 20;
	public final int backLeftTurnID = 30;
	public final int backRightTurnID = 41;

	// IDs for Drive Motors
	public final int frontLeftDriveID = 11;
	public final int frontRightDriveID = 21;
	public final int backLeftDriveID = 31;
	public final int backRightDriveID = 40; // todo: flip 41 and 40 (to match)

	// Analog Encoder ID
	public final int frontLeftEncoderID = 1;
	public final int frontRightEncoderID = 2;
	public final int backLeftEncoderID = 0;
	public final int backRightEncoderID = 3;

	// Offset of analog to make encoders face forward
	public final int frontLeftEncoderOffset = 0;
	public final int frontRightEncoderOffset = 0;
	public final int backLeftEncoderOffset = 0;
	public final int backRightEncoderOffset = 0;
}