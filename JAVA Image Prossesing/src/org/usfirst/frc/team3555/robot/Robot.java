package org.usfirst.frc.team3555.robot;

import static com.ni.vision.NIVision.*;

import java.awt.image.BufferedImage;
import java.awt.image.ColorModel;
import java.awt.image.DataBufferByte;
import java.awt.image.Raster;
import java.awt.image.WritableRaster;
import java.io.File;
import java.io.IOException;
import java.nio.ByteBuffer;

import com.ni.vision.NIVision.AxisOrientation;
import com.ni.vision.NIVision.CalibrationUnit;
import com.ni.vision.NIVision.ColorMode;
import com.ni.vision.NIVision.CoordinateSystem;
import com.ni.vision.NIVision.DrawMode;
import com.ni.vision.NIVision.GetImageSizeResult;
import com.ni.vision.NIVision.GridDescriptor;
import com.ni.vision.NIVision.Image;
import com.ni.vision.NIVision.ImageType;
import com.ni.vision.NIVision.MeasurementType;
import com.ni.vision.NIVision.ParticleFilterCriteria2;
import com.ni.vision.NIVision.ParticleFilterOptions2;
import com.ni.vision.NIVision.Point;
import com.ni.vision.NIVision.PointFloat;
import com.ni.vision.NIVision.ScalingMethod;

import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.vision.USBCamera;

public class Robot extends SampleRobot {
	private final int MINIMUM_SCORE = 750;
	private final String TEMPLATE_PATH = "/Rectangle Shape Template.png";

	private USBCamera camera;

	private Joystick joyOp;
	
	public Robot() {
		joyOp = new Joystick(1);
		camera = new USBCamera("cam1");
		camera.startCapture();
	}

	public void operatorControl() {
		int lastImage = 0;
		int index = 0;
		camera.setSize(320, 240); 
		
		while (isOperatorControl() && isEnabled()) {
			Image maskImage = imaqCreateImage(ImageType.IMAGE_U8, 7);			
			Image image = imaqCreateImage(ImageType.IMAGE_U8, 7);
			camera.getImage(image);
			imaqDuplicate(maskImage, image);

			SmartDashboard.putNumber("Index: ", index ++);
			
			GetImageSizeResult size = imaqGetImageSize(image);
			SmartDashboard.putString("Size: ", size.width + ", " + size.height);
			
			Image compositThreshold = imaqCreateImage(ImageType.IMAGE_U8, 7);
			thresholdProsses(compositThreshold, image);
			
			Image filterImage = imaqCreateImage(ImageType.IMAGE_U8, 7);
			removeSmallParicles(filterImage, compositThreshold, 150);
			imaqRejectBorder(filterImage, filterImage, 1);
			imaqConvexHull(filterImage, filterImage, 1);
			removeElogatedParticles(filterImage, filterImage, 3);
			
			imaqMask(maskImage, maskImage, filterImage);
			
			GridDescriptor gridDescriptor = new GridDescriptor(1, 1, CalibrationUnit.UNDEFINED);
			CoordinateSystem coordinateSystem = new CoordinateSystem(new PointFloat(0, 0), 0, AxisOrientation.INDIRECT);
			imaqSetSimpleCalibration(filterImage, ScalingMethod.SCALE_TO_FIT, 1, gridDescriptor, coordinateSystem);
			
//			GetCalibrationInfoReport info = imaqCalibrationGetCalibrationInfo(filterImage, 0);
//			SmartDashboard.putString("info: ", info.calibrationRoi.toString());
//			info.free();
			
			int particleCount = imaqCountParticles(filterImage, 1);
			SmartDashboard.putNumber("Number of Particles: ", particleCount);
			AABB[] particles = new AABB[particleCount];
			for(int i = 0; i < particleCount; i ++) {
				particles[i] = AABB.loadAABB(filterImage, true, i);
			}
			
			long startTime = System.currentTimeMillis();
//			imaqWriteFile(maskImage, "/passThrough.bmp", RGB_WHITE);
			imaqWriteBMPFile(filterImage, "/passThrough1.bmp", 0, RGB_BLACK);
			ByteBuffer readImage = File_IO.read("/passThrough.bmp"); readImage.reset();
			SimpleBinaryImage simpleImage = SimpleBinaryImage.loadBMPImage(readImage);
			SmartDashboard.putNumber("Time: ", System.currentTimeMillis() - startTime);
			
			for(AABB aabb : particles) {
				Sample[] corner = findBoundingBoxCorner(aabb, simpleImage, 16);
				for(int i = 0; i < corner.length; i ++) {
					imaqDrawLineOnImage(filterImage, filterImage, DrawMode.DRAW_VALUE, 
							new Point(corner[i].getCenterX(), corner[i].getCenterY()), 
							new Point(corner[(i + 1) % corner.length].getCenterX(), 
									corner[(i + 1) % corner.length].getCenterY()), 255);
				}
			}
			
			boolean newImageUse = false;
			for(int i = 1; i < 12; i ++) {
				if(i == 4) continue;
				if(!joyOp.getRawButton(i)) continue;
				if(i == 1) { CameraServer.getInstance().setImage(image); lastImage = i; newImageUse = true;}

				if(i == 7) { CameraServer.getInstance().setImage(compositThreshold); lastImage = i; newImageUse = true;}

				if(i == 10) { CameraServer.getInstance().setImage(maskImage); lastImage = i; newImageUse = true;}
				if(i == 11) { CameraServer.getInstance().setImage(filterImage); lastImage = i; newImageUse = true;}
			}
			
			SmartDashboard.putNumber("Display Image: ", lastImage);
			
			if(!newImageUse) {
				if(lastImage == 1) { CameraServer.getInstance().setImage(image);}

				if(lastImage == 7) { CameraServer.getInstance().setImage(compositThreshold);}

				if(lastImage == 10) { CameraServer.getInstance().setImage(maskImage);}
				if(lastImage == 11) { CameraServer.getInstance().setImage(filterImage);}
			}
			
			SmartDashboard.putString("Image Key Map: ", ""
					+ "\n1: image "
					
					+ "\n7: compositThreshold"
					
					+ "\n10: filterImage"
					+ "\n11: filterImage"
				);
			
			image.free();
			maskImage.free();
			
			compositThreshold.free();
			filterImage.free();
		}
	}
	
	private Image thresholdProsses(Image dest, Image source) {
		Image redComponent = imaqCreateImage(ImageType.IMAGE_U8, 7);
		Image greenComponent = imaqCreateImage(ImageType.IMAGE_U8, 7);
		Image blueComponent = imaqCreateImage(ImageType.IMAGE_U8, 7);
		imaqExtractColorPlanes(source, ColorMode.RGB, redComponent, greenComponent, blueComponent);
		
		Image lumComponent = imaqCreateImage(ImageType.IMAGE_U8, 7);
		imaqExtractColorPlanes(source, ColorMode.HSV, null, null, lumComponent);
		
		Image finalCompost = imaqCreateImage(ImageType.IMAGE_U8, 7);
		imaqOr(finalCompost, redComponent, greenComponent);
		imaqAnd(finalCompost, finalCompost, blueComponent);
		imaqNand(finalCompost, finalCompost, lumComponent);
		
		imaqThreshold(dest, finalCompost, 0, 100, 1, 255);
		
		redComponent.free();
		greenComponent.free();
		blueComponent.free();
		lumComponent.free();
		
		finalCompost.free();
		
		return dest;
	}

	public Image removeSmallParicles(Image dest, Image source, int minParimeter) {
		ParticleFilterCriteria2[] particleCriteria = new ParticleFilterCriteria2[] { new ParticleFilterCriteria2(
				MeasurementType.MT_PERIMETER, 0, minParimeter, 0, 0) };
		
		ParticleFilterOptions2 particleFilterOptions = new ParticleFilterOptions2(1, 0, 0, 1);
		imaqParticleFilter4(dest, source, particleCriteria, particleFilterOptions, null);
		particleFilterOptions.free();
		
		return dest;
	}
	
	public Image removeElogatedParticles(Image dest, Image source, int maxEloncation) {
		ParticleFilterCriteria2[] particleCriteria = new ParticleFilterCriteria2[] { new ParticleFilterCriteria2(
				MeasurementType.MT_ELONGATION_FACTOR, maxEloncation, 65536, 0, 0) };
		
		ParticleFilterOptions2 particleFilterOptions = new ParticleFilterOptions2(1, 0, 0, 1);
		imaqParticleFilter4(dest, source, particleCriteria, particleFilterOptions, null);
		particleFilterOptions.free();
		
		return dest;
	}
	
	
	public Sample[] findBoundingBoxCorner(AABB aabb, SimpleBinaryImage image, int sampleSize) {
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
	
	public static class SimpleBinaryImage {
		private int width, height;
		private boolean[] imageData;
		
		private SimpleBinaryImage(int width, int height) {
			this.width = width;
			this.height = height;
			
			imageData = new boolean[width * height];
		}
		
		public static SimpleBinaryImage loadBMPImage(ByteBuffer buffer) {
			SimpleBMPImage.BMP_Header.read(buffer);
			SimpleBMPImage.BMP_Info_Header info_Header = SimpleBMPImage.BMP_Info_Header.read(buffer);
		    int padding = (4 - (info_Header.width * 3) % 4) % 4;
			
		    SimpleBinaryImage binaryImage = new SimpleBinaryImage(info_Header.width, Math.abs(info_Header.height));
			
			for(int y = 0; y < binaryImage.height; y ++) {
				for(int x = 0; x < binaryImage.width; x ++) {
					SimpleBMPImage.RGB_Triple rgb = SimpleBMPImage.RGB_Triple.read(buffer);
					binaryImage.imageData[x + y * binaryImage.width] = 
							rgb.getRed() + rgb.getGreen() + rgb.getBlue() > 0;
				}
				
				buffer.get(new byte[padding], 0, padding);
			}
			
			return binaryImage;
		}

		public int getWidth() { return width; }
		public int getHeight() { return height; }

		public boolean[] getImageData() { return imageData; }
		public boolean get(int x, int y) { return imageData[x + y * width]; }
	}
	
	public static class SimpleBMPImage {
		private int width, height;
		private int[] imageData;
		
		public SimpleBMPImage(ByteBuffer buffer) {
			BMP_Header.read(buffer);
			BMP_Info_Header info_Header = BMP_Info_Header.read(buffer);
		    int padding = (4 - (info_Header.width * 3) % 4) % 4;
			
			this.width = Math.abs(info_Header.width);
			this.height = Math.abs(info_Header.height);
			this.imageData = new int[width * height];
			
			for(int y = 0; y < height; y ++) {
				for(int x = 0; x < width; x ++) {
					RGB_Triple rgb = RGB_Triple.read(buffer);
					imageData[x + y * width] = rgb.getRGB();
				}
				
				buffer.get(new byte[padding], 0, padding);
			}
		}
		
		public static class RGB_Triple {
			private byte red;
			private byte green;
			private byte blue;
			
			private RGB_Triple() {}
			
			public static RGB_Triple read(ByteBuffer buffer) {
				RGB_Triple rgb = new RGB_Triple();
				
				rgb.red = buffer.get();
				rgb.green = buffer.get();
				rgb.blue = buffer.get();
				
				return rgb;
			}
			
			public byte getRed() { return red; }
			public byte getGreen() { return green; }
			public byte getBlue() { return blue; }

			public int getRGB() {
				return red << 16 | green << 8 | blue;
			}
		}

		@SuppressWarnings("unused")
		public static class BMP_Header {
			private short type;
			private int size;
			private short reserved1;
			private short reserved2;
			private int offBit;
			
			private BMP_Header() {}
			
			public static BMP_Header read(ByteBuffer buffer) {
				BMP_Header header = new BMP_Header();
				
				header.type = readShort(buffer);
				header.size = readInt(buffer);
				header.reserved1 = readShort(buffer);
				header.reserved2 = readShort(buffer);
				header.offBit = readInt(buffer);
				
				return header;
			}
			
			private static short readShort(ByteBuffer buffer) {
				byte[] bytes = new byte[2];
				buffer.get(bytes, 0, bytes.length);
				short value = 0;
				for(int i = 0; i < bytes.length; i ++)
					value |= bytes[i] << 8*i;
				return value;
			}
			
			private static int readInt(ByteBuffer buffer) {
				byte[] bytes = new byte[4];
				buffer.get(bytes, 0, bytes.length);
				int value = 0;
				for(int i = 0; i < bytes.length; i ++)
					value |= bytes[i] << 8*i;
				return value;
			}
		}

		@SuppressWarnings("unused")
		public static class BMP_Info_Header {
			private int size; 
		    private int width; 
		    private int height; 
		    private short planes; 
		    private short bitCount; 
		    private int compression; 
		    private int sizeImage; 
		    private int xPelsPerMeter; 
		    private int yPelsPerMeter; 
		    private int clrUsed; 
		    private int clrImportant; 
			
			private BMP_Info_Header() {}
			
			public static BMP_Info_Header read(ByteBuffer buffer) {
				BMP_Info_Header header = new BMP_Info_Header();
				
				header.size = readInt(buffer);
				header.width = readInt(buffer);
				header.height = readInt(buffer);
				header.planes = readShort(buffer);
				header.bitCount = readShort(buffer);
				header.compression = readInt(buffer);
				header.sizeImage = readInt(buffer);
				header.xPelsPerMeter = readInt(buffer); 
				header.yPelsPerMeter = readInt(buffer); 
				header.clrUsed = readInt(buffer);
				header.clrImportant  = readInt(buffer);
				
				return header;
			}
			
			private static short readShort(ByteBuffer buffer) {
				byte[] bytes = new byte[2];
				buffer.get(bytes, 0, bytes.length);
				short value = 0;
				for(int i = 0; i < bytes.length; i ++)
					value |= bytes[i] << 8*i;
				return value;
			}
			
			private static int readInt(ByteBuffer buffer) {
				byte[] bytes = new byte[4];
				buffer.get(bytes, 0, bytes.length);
				int value = 0;
				for(int i = 0; i < bytes.length; i ++)
					value |= bytes[i] << 8*i;
				return value;
			}
		}

		public int getWidth() { return width; }
		public int getHeight() { return height; }
		
		public int getRGB(int x, int y) {
			return imageData[x + y * width];
		}
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

//if(joyOp.getRawButton(2)){
//int index = 0;
//try {
//	while(true) {
//		Image indexTest = imaqCreateImage(ImageType.IMAGE_U8, 7);
//		imaqReadFile(indexTest, "/image" + index + ".jpg");
//		indexTest.free();
//		
//		index ++;
//	}
//} catch(Exception e) {}
//	
//imaqWriteFile(image, "/image" + index + ".jpg", RGB_WHITE);
//return;
//}

//public void findCorner(AABB aabb, BufferedImage image, int sampleSize, float halfMark, float deviation) {
//	// This method finds the location of a corner a shape by quickly and inaccurately shifting to the edge of
//	// the image, then slowly shifting back to the edge until it has reached a pre-specified percentage +/- a deviation
//	// This same process is then repeated by moving up the shape, but this time the percentage is split in half
//	
//	boolean[][] sample = new boolean[sampleSize][sampleSize];
//	
//	int x = aabb.getCenterX(), y = aabb.getCenterY();
//	float percentage = 0;
//
//	// ---------------------------------------------- ---------- ---------------------------------------------- \\
//	// ---------------------------------------------- X Stepping ---------------------------------------------- \\
//	
//	// Steps over the sample area until a certain percentage of the sample is selected +/- a deviation
//	// Or until 0 percent of the sample is selected
//	do {
//		x += sampleSize;  // Moves the sample section to the Right by sampleSize
//		
//		for(int xScan = 0; xScan < sampleSize; xScan ++) {
//		// Samples a column of the image from: [-sampleSize/2 + y] to [sampleSize/2 + y]
//		for(int yScan = -sampleSize/2; yScan < sampleSize/2; yScan ++) {
//			
//			// Skip any parts outside of the Image
//			if(yScan + y >= image.getHeight() || yScan + y < 0 || xScan + x < image.getWidth()) {
//				sample[xScan][yScan + sampleSize/2] = false; 
//				continue;
//			}
//												// Check if the "Green?" component of the color is > 0
//			sample[xScan][yScan + sampleSize/2] = (image.getRGB(x + xScan, y + yScan) & 0xFF00) > 0;
//			
//			// The component is check if it is over 0 because the image is a Gray-Scale formated to represent
//			// A Binary-Image were a value of 0 is "false" and a value of 1 is "true"
//			
//			// In a Gray-Scale Image all color components except Alpha are set to the same value
//			
//			// Color component part "0xFF00" is used to avoid the Alpha component
//			// This is done because most common color formats use ARGB or RGBA
//			// By selecting one of the two middle components the Alpha is completely avoided
//		}}
//		
//		
//		// TODO: Check for gaps in the sample, which can invalidating the percentage calculation  
//		percentage = calcPercentage(sample);
//	} while(Math.abs(percentage - halfMark) > deviation && percentage > 0);
//
//	// ---------------------------------------------- ---------- ---------------------------------------------- \\
//	// ---------------------------------------------- X Tweaking ---------------------------------------------- \\
//	
//	// Slowly shifts back the checking region until it finds a valid section
//	// Or the selection falls nearly completely back into the full shape
//	while(Math.abs(percentage - halfMark) > deviation && percentage > halfMark - deviation) {
//		x --;  // Slowly move the sample section to the Left by 1 pixel at a time
//		
//		for(int xScan = 0; xScan < sampleSize; xScan ++) {
//		for(int yScan = -sampleSize/2; yScan < sample/2; yScan ++) {
//			
//			// Skip any parts outside of the Image
//			if(yScan + y >= image.getHeight() || yScan + y < 0 || xScan + x < image.getWidth()) {
//				sample[xScan][yScan + sampleSize/2] = false; 
//				continue;
//			}
//												// Check if the "Green?" component of the color is > 0
//			sample[xScan][yScan + sampleSize/2] = (image.getRGB(x + xScan, y + yScan) & 0xFF00) > 0;
//		}}
//		
//		// TODO: Check for gaps in the sample, which can invalidating the percentage calculation  
//		percentage = calcPercentage(sample);
//	}
//	
//	// ---------------------------------------------- ---------- ---------------------------------------------- \\
//	// ---------------------------------------------- Y Stepping ---------------------------------------------- \\
//	
//	// Steps over sampleSize section, similarly to how X-Stepping moves, but this time along the Y-Axis
//	// Also the percentage amount is halved, and if the percentage exceeds halfMark +/- deviation
//	// X is recalculated to account for what is most likely a slanted Edge
//	do {
//		x += sampleSize;  // Moves the sample section to the Right by sampleSize
//		
//		for(int xScan = 0; xScan < sampleSize; xScan ++) {
//		// Samples a column of the image from: [-sampleSize/2 + y] to [sampleSize/2 + y]
//		for(int yScan = -sampleSize/2; yScan < sampleSize/2; yScan ++) {
//			
//			// Skip any parts outside of the Image
//			if(yScan + y >= image.getHeight() || yScan + y < 0 || xScan + x < image.getWidth()) {
//				sample[xScan][yScan + sampleSize/2] = false; 
//				continue;
//			}
//												// Check if the "Green?" component of the color is > 0
//			sample[xScan][yScan + sampleSize/2] = (image.getRGB(x + xScan, y + yScan) & 0xFF00) > 0;
//			
//			// The component is check if it is over 0 because the image is a Gray-Scale formated to represent
//			// A Binary-Image were a value of 0 is "false" and a value of 1 is "true"
//			
//			// In a Gray-Scale Image all color components except Alpha are set to the same value
//			
//			// Color component part "0xFF00" is used to avoid the Alpha component
//			// This is done because most common color formats use ARGB or RGBA
//			// By selecting one of the two middle components the Alpha is completely avoided
//		}}
//		
//		
//		// TODO: Check for gaps in the sample, which can invalidating the percentage calculation  
//		percentage = calcPercentage(sample);
//	} while(Math.abs(percentage - halfMark) > deviation && percentage > 0);
//
//	// ---------------------------------------------- ---------- ---------------------------------------------- \\
//	// ---------------------------------------------- Y Tweaking ---------------------------------------------- \\
//	
//	// Slowly shifts back the checking region until it finds a valid section
//	// Or the selection falls nearly completely back into the full shape
//	while(Math.abs(percentage - halfMark) > deviation && percentage > halfMark - deviation) {
//		x --;  // Slowly move the sample section to the Left by 1 pixel at a time
//		
//		for(int xScan = 0; xScan < sampleSize; xScan ++) {
//		for(int yScan = -sampleSize/2; yScan < sampleSize/2; yScan ++) {
//			
//			// Skip any parts outside of the Image
//			if(yScan + y >= image.getHeight() || yScan + y < 0 || xScan + x < image.getWidth()) {
//				sample[xScan][yScan + sampleSize/2] = false; 
//				continue;
//			}
//												// Check if the "Green?" component of the color is > 0
//			sample[xScan][yScan + sampleSize/2] = (image.getRGB(x + xScan, y + yScan) & 0xFF00) > 0;
//		}}
//		
//		// TODO: Check for gaps in the sample, which can invalidating the percentage calculation  
//		percentage = calcPercentage(sample);
//	}
//}