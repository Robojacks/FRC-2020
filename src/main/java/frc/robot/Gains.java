/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * A place to store PID gains and feedforward gains
 */
public class Gains {

	// PID Constants
	public static class leftDrive {
		public static double kP = 0.1;
		public static double kI = 0;
		public static double kD = 0;
	}

	public static class rightDrive {
		public static double kP = 0.1;
		public static double kI = 0;
		public static double kD = 0;
	}

	public static class shooterFeedforward {
		public static double ks = 0;
		public static double kv = 0;
	}

	public static class shooterPID {
		public static double kP = 0.0015;
		public static double kI = 0;
		public static double kD = 0.3;
		public static double kF = 0.0005;
	}


	public static class angleCorrection {
		public static double kP = 0.05;
		public static double kI = 0;
		public static double kD = 5;
	}

	// Ramsete controller constants
	public static class Ramsete {
		public static final double kBeta = 2.0;
		public static final double kZeta = 0.7;
	}

	// Feed Forward Constants
	public static class driveFeedforward {
		public static final double ks = 0;
		public static final double kv = 0;
		public static final double ka = 0;
	}

}
