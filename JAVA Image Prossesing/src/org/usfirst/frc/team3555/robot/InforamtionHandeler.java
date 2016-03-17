package org.usfirst.frc.team3555.robot;

import org.usfirst.frc.team3555.robot.ShapeMatcher.Sample;

public class InforamtionHandeler {
	private static final double TOP_WIDTH = 10;
	private static final double BOTTOM_WIDTH = 10;
	private static final double LEFT_HEIGHT = 10;
	private static final double RIGHT_HEIGHT = 10;
	
	public static int scoreRectangle(Sample upperLeft, Sample upperRight, Sample lowerLeft, Sample lowerRight) {
		double topWidth = upperRight.getCenterX() - upperLeft.getCenterX();
		double bottomWidth = lowerRight.getCenterX() - lowerLeft.getCenterX();
		double leftHeight = Math.abs(upperLeft.getCenterY() - lowerLeft.getCenterY());
		double rightHeight = Math.abs(upperRight.getCenterY() - lowerRight.getCenterY());
		
		// A Perfict score is 4000
		return (int) ((
				topWidth / TOP_WIDTH +
				bottomWidth / BOTTOM_WIDTH +
				leftHeight / LEFT_HEIGHT +
				rightHeight / RIGHT_HEIGHT
			) * 1000);
	}
}
