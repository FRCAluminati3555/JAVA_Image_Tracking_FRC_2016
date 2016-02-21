package org.usfirst.frc.team3555.robot;

import static com.ni.vision.NIVision.imaqMeasureParticle;

import org.usfirst.frc.team3555.robot.SimpleImage.SimpleBinaryImage;

import com.ni.vision.NIVision.Image;
import com.ni.vision.NIVision.MeasurementType;

public class ShapeMatcher {
	public static Sample[] findBoundingBoxCorner(AABB aabb, SimpleBinaryImage image, int sampleSize) {
		Sample[] samples = new Sample[4];
		for(int i = 0; i < samples.length; i ++) {
			samples[i] = new Sample(sampleSize, aabb.getCenterX(), aabb.getCenterY());
			samples[i].bindImage(image);
		}
		
		samples[0].findCorner(Direction.Right, Direction.Up, .5f, .25f, .02f, .2f, (int) Math.ceil((double) sampleSize / 5.0));
		samples[1].findCorner(Direction.Right, Direction.Down, .5f, .25f, .02f, .2f, (int) Math.ceil((double) sampleSize / 5.0)); 
		samples[0].findCorner(Direction.Left, Direction.Down, .5f, .25f, .02f, .2f, (int) Math.ceil((double) sampleSize / 5.0));
		samples[1].findCorner(Direction.Left, Direction.Up, .5f, .25f, .02f, .2f, (int) Math.ceil((double) sampleSize / 5.0)); 
	
		return samples;
	}
	
	public static class Sample {
		private SimpleBinaryImage image;
		private int sampleSize;
		private boolean[][] sampleData;
		private int x, y;
		
		public Sample(int sampleSize, int x, int y) {
			this.sampleSize = sampleSize;
			this.sampleData = new boolean[sampleSize][sampleSize];
			
			this.x = x;
			this.y = y;
		}
		
		public float calcPercentage() {
			float total = 0.0f;
			for(boolean[] array : sampleData) {
				for(boolean bool : array) {
					total += bool ? 1 : 0;
				}
			}
			
			return total / (float)(sampleSize * sampleSize);				
		}
		
		public float calcPercentageOnEdge(Direction direction, int edgeSize) {
			int x = 0, y = 0, xMod = 0, yMod = 0;
			
			switch (direction) {
				case Down: y = sampleSize - edgeSize; xMod = 1; break;
				case Left: yMod = 1; break;
					
				case Right: x = sampleSize - edgeSize; yMod = 1; break;
				case Up: xMod = 1; break;
			}
			
			float total = 0;
			for(int j = 0; j < edgeSize; j ++) {
			for(int i = 0; i < sampleSize; i ++) {
				total += sampleData[x + i * xMod + j * xMod == 0 ? 1 : 0][y + i * yMod + j * yMod == 0 ? 1 : 0] ? 1 : 0;
			}}
			
			return total / (float) sampleSize;
		}
		
		/**
		 * Moves the sample in a Direction by sampleSize 
		 */
		public void step(Direction direction) {
			if(direction.isVertical())
				y += direction.getModifier() * sampleSize;
			else
				x += direction.getModifier() * sampleSize;			
		}
		
		/**
		 * Moves the sample in a Direction by one Pixel 
		 */
		public void nudge(Direction direction) {
			if(direction.isVertical())
				y += direction.getModifier();
			else
				x += direction.getModifier();			
		}
		
		public float sample(OverflowHandel handel) {
			if(image == null) throw new IllegalStateException("No Image bound to Sample. Can not sample");
			
			float total = 0.0f;
			for(int x = 0; x < sampleSize; x ++) {
			for(int y = 0; y < sampleSize; y ++) {
				if(this.x + x >= image.getWidth() || this.y + y >= image.getHeight() || this.x + x < 0 || this.y + y < 0) 
					sampleData[x][y] = handel.handelOverflow(this);
				else
					sampleData[x][y] = image.get(x + this.x, y + this.y);
					
				total += sampleData[x][y] ? 1 : 0;	
			}}
			
			return total / (float)(sampleSize * sampleSize);
		}
		
		public void findEdge(Direction direction, float targetPercantage, float deviation, float boarderPercentage, int boarderSize) {
			float percenatge = 0;
			sample(OverflowHandel.Zero_On_Overflows);

			// Stepping Step
			while(calcPercentageOnEdge(direction, boarderSize) > boarderPercentage) {
				step(direction);
				percenatge = sample(OverflowHandel.Zero_On_Overflows);
			}
						
			// Tweaking Step
			float differance = percenatge - targetPercantage;
			while(Math.abs(differance) > deviation) {
				if(differance > 0)
					nudge(direction);
				else 
					nudge(direction.getOppsite());
				percenatge = sample(OverflowHandel.Zero_On_Overflows);
				differance = percenatge - targetPercantage;
			}
		}
		
		public void findCorner(Direction primaryDirection, Direction secondaryDirection, float primaryPercenatge, float secondaryPercentage, float deviation, float boarderPercentage, int boarderSize) {
			findEdge(primaryDirection, primaryPercenatge, deviation, boarderPercentage, boarderSize);
			findEdge(secondaryDirection, secondaryPercentage, deviation, boarderPercentage, boarderSize);
		}
		
		// ----------------------------------- -------------------- ----------------------------------- \\
		// ----------------------------------- Accessor / Modifiers ----------------------------------- \\
		
		/**
		 * Sets the image that the sample will sample from
		 * @param image The image to use
		 */
		public void bindImage(SimpleBinaryImage image) {
			this.image = image;
		}
		
		public void clearSample() {
			for(int x = 0; x < sampleSize; x ++) {
			for(int y = 0; y < sampleSize; y ++) {
				sampleData[x][y] = false;
			}}
		}
		
		public int getSampleSize() { return sampleSize; }
		public boolean[][] getSampleData() { return sampleData; }

		public int getX() { return x; }
		public int getY() { return y; }
		
		public int getCenterX() { return x + sampleSize / 2; }
		public int getCenterY() { return y + sampleSize / 2; }
		
		public static enum OverflowHandel {
			Crash_On_Overflow, Zero_On_Overflows, Zero_All_On_Overflow, One_On_Overflows;
			
			public boolean handelOverflow(Sample sample) {
				switch(this) {
					case Crash_On_Overflow: 
						throw new IndexOutOfBoundsException("Overflow handle " + toString());
					case Zero_All_On_Overflow: sample.clearSample(); 
					case Zero_On_Overflows: return false;
					case One_On_Overflows: return true;
					
					default: return false;
				}
			}
		}
	}
	
	public static enum Direction {
		Right(1), Left(-1), Up(-1), Down(1);
		
		private int modifier;
		private Direction(int modifier) {
			this.modifier = modifier;
		}
		
		public int getModifier() { return modifier; }
		public boolean isVertical() { return this == Up || this == Down; }
		
		public Direction getOppsite() {
			switch(this) {
				case Down: return Up;
				case Left: return Right;
				case Right: return Left;
				case Up: return Down;
				
				default: return null;
			}
		}
		
		public static Direction getDirection(int modifer, boolean isVerticle) {
			if(isVerticle) {
				if(modifer < 0) return Up;
				else 			return Down;
			} else {
				if(modifer < 0) return Left;
				else 			return Right;
			}
		}
	}
	
	public static class AABB {
		private int x, y;
		private int width, height;
		private int centerX, centerY;
		
		public AABB(int x, int y, int width, int height) {
			this.x = x;
			this.y = y;
			this.width = width;
			this.height = height;
			
			this.centerX = x + width / 2;
			this.centerY = y + height / 2;
		}
		
		public static AABB loadAABB(Image image, boolean connectivity8, int particleIndex) {
			double width = imaqMeasureParticle(image, particleIndex, connectivity8 ? 1 : 0, MeasurementType.MT_BOUNDING_RECT_WIDTH);
			double height = imaqMeasureParticle(image, particleIndex, connectivity8 ? 1 : 0, MeasurementType.MT_BOUNDING_RECT_HEIGHT);
			double x = imaqMeasureParticle(image, particleIndex, connectivity8 ? 1 : 0, MeasurementType.MT_BOUNDING_RECT_LEFT);
			double y = imaqMeasureParticle(image, particleIndex, connectivity8 ? 1 : 0, MeasurementType.MT_BOUNDING_RECT_TOP);
			
			return new AABB((int) x, (int) y, (int) width, (int) height);
		}

		public int getX() { return x; }
		public int getY() { return y; }
		public int getWidth()  { return width; }
		public int getHeight() { return height; }
		public int getCenterX() { return centerX; }
		public int getCenterY() { return centerY; }
	}
}
