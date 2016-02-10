package org.usfirst.frc.team3555.robot;

import static com.ni.vision.NIVision.RGB_WHITE;
import static com.ni.vision.NIVision.imaqAnd;
import static com.ni.vision.NIVision.imaqConvexHull;
import static com.ni.vision.NIVision.imaqCountParticles;
import static com.ni.vision.NIVision.imaqCreateImage;
import static com.ni.vision.NIVision.imaqDuplicate;
import static com.ni.vision.NIVision.imaqExtractColorPlanes;
import static com.ni.vision.NIVision.imaqGetImageSize;
import static com.ni.vision.NIVision.imaqMask;
import static com.ni.vision.NIVision.imaqMatchShape;
import static com.ni.vision.NIVision.imaqMeasureParticle;
import static com.ni.vision.NIVision.imaqNand;
import static com.ni.vision.NIVision.imaqOr;
import static com.ni.vision.NIVision.imaqParticleFilter4;
import static com.ni.vision.NIVision.imaqReadFile;
import static com.ni.vision.NIVision.imaqRejectBorder;
import static com.ni.vision.NIVision.imaqThreshold;
import static com.ni.vision.NIVision.imaqWriteFile;

import java.awt.Toolkit;
import java.awt.image.BufferedImage;

import com.ni.vision.NIVision.ColorMode;
import com.ni.vision.NIVision.GetImageSizeResult;
import com.ni.vision.NIVision.Image;
import com.ni.vision.NIVision.ImageType;
import com.ni.vision.NIVision.MatchShapeResult;
import com.ni.vision.NIVision.MeasurementType;
import com.ni.vision.NIVision.ParticleFilterCriteria2;
import com.ni.vision.NIVision.ParticleFilterOptions2;
import com.ni.vision.NIVision.ShapeReport;

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
		while (isOperatorControl() && isEnabled()) {
			Image maskImage = imaqCreateImage(ImageType.IMAGE_U8, 7);			
			Image image = imaqCreateImage(ImageType.IMAGE_U8, 7);
			camera.getImage(image);
			imaqDuplicate(maskImage, image);
			
			if(joyOp.getRawButton(2)){
				int index = 0;
				try {
					while(true) {
						Image indexTest = imaqCreateImage(ImageType.IMAGE_U8, 7);
						imaqReadFile(indexTest, "/image" + index + ".jpg");
						indexTest.free();
						
						index ++;
					}
				} catch(Exception e) {}
					
				imaqWriteFile(image, "/image" + index + ".jpg", RGB_WHITE);
				return;
			}
			
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

			int particleCount = imaqCountParticles(filterImage, 1);
			AABB[] particles = new AABB[particleCount];
			for(int i = 0; i < particleCount; i ++) {
				particles[i] = AABB.loadAABB(filterImage, true, i);
			}
			
			imaqWriteFile(filterImage, "/passThroughImage.jpg", RGB_WHITE);
			java.awt.Image javaImage = Toolkit.getDefaultToolkit().getImage("/passThroughImage.jpg");
//			javaImage.
			
			
			
			
			
			Image shapeImage = imaqCreateImage(ImageType.IMAGE_U8, 7);
			imaqDuplicate(shapeImage, filterImage);

			Image imageTemplate = imaqCreateImage(ImageType.IMAGE_U8, 7);
			imaqReadFile(imageTemplate, TEMPLATE_PATH);
			imaqThreshold(imageTemplate, imageTemplate, 1, 255, 1, 1);
			
			MatchShapeResult shapeReport = imaqMatchShape(shapeImage, shapeImage, imageTemplate, 1, 1, 0.5);

			int numOfMatches = 0;
			for(ShapeReport report : shapeReport.array) {
				if(report.score >= MINIMUM_SCORE) {
					numOfMatches ++;
				}
			}

			SmartDashboard.putNumber("Number of Matches: ", numOfMatches);

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
			imageTemplate.free();
			shapeImage.free();
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
	
	public void findCorner(AABB aabb, BufferedImage image, int sampleSize, float halfMark, float deviation) {
		// This method finds the location of a corner a shape by quickly and inaccurately shifting to the edge of
		// the image, then slowly shifting back to the edge until it has reached a pre-specified percentage +/- a deviation
		// This same process is then repeated by moving up the shape, but this time the percentage is split in half
		
		boolean[][] sample = new boolean[sampleSize][sampleSize];
		
		int x = aabb.getCenterX(), y = aabb.getCenterY();
		float percentage = 0;

		// ---------------------------------------------- ---------- ---------------------------------------------- \\
		// ---------------------------------------------- X Stepping ---------------------------------------------- \\
		
		// Steps over the sample area until a certain percentage of the sample is selected +/- a deviation
		// Or until 0 percent of the sample is selected
		do {
			x += sampleSize;  // Moves the sample section to the Right by sampleSize
			
			for(int xScan = 0; xScan < sampleSize; xScan ++) {
			// Samples a column of the image from: [-sampleSize/2 + y] to [sampleSize/2 + y]
			for(int yScan = -sampleSize/2; yScan < sampleSize/2; yScan ++) {
				
				// Skip any parts outside of the Image
				if(yScan + y >= image.getHeight() || yScan + y < 0 || xScan + x < image.getWidth()) {
					sample[xScan][yScan + sampleSize/2] = false; 
					continue;
				}
													// Check if the "Green?" component of the color is > 0
				sample[xScan][yScan + sampleSize/2] = (image.getRGB(x + xScan, y + yScan) & 0xFF00) > 0;
				
				// The component is check if it is over 0 because the image is a Gray-Scale formated to represent
				// A Binary-Image were a value of 0 is "false" and a value of 1 is "true"
				
				// In a Gray-Scale Image all color components except Alpha are set to the same value
				
				// Color component part "0xFF00" is used to avoid the Alpha component
				// This is done because most common color formats use ARGB or RGBA
				// By selecting one of the two middle components the Alpha is completely avoided
			}}
			
			
			// TODO: Check for gaps in the sample, which can invalidating the percentage calculation  
			percentage = calcPercentage(sample);
		} while(Math.abs(percentage - halfMark) > deviation && percentage > 0);

		// ---------------------------------------------- ---------- ---------------------------------------------- \\
		// ---------------------------------------------- X Tweaking ---------------------------------------------- \\
		
		// Slowly shifts back the checking region until it finds a valid section
		// Or the selection falls nearly completely back into the full shape
		while(Math.abs(percentage - halfMark) > deviation && percentage > halfMark - deviation) {
			x --;  // Slowly move the sample section to the Left by 1 pixel at a time
			
			for(int xScan = 0; xScan < sampleSize; xScan ++) {
			for(int yScan = -sampleSize/2; yScan < sampleSize/2; yScan ++) {
				
				// Skip any parts outside of the Image
				if(yScan + y >= image.getHeight() || yScan + y < 0 || xScan + x < image.getWidth()) {
					sample[xScan][yScan + sampleSize/2] = false; 
					continue;
				}
													// Check if the "Green?" component of the color is > 0
				sample[xScan][yScan + sampleSize/2] = (image.getRGB(x + xScan, y + yScan) & 0xFF00) > 0;
			}}
			
			// TODO: Check for gaps in the sample, which can invalidating the percentage calculation  
			percentage = calcPercentage(sample);
		}
		
		// ---------------------------------------------- ---------- ---------------------------------------------- \\
		// ---------------------------------------------- Y Stepping ---------------------------------------------- \\
		
		// Steps over sampleSize section, similarly to how X-Stepping moves, but this time along the Y-Axis
		// Also the percentage amount is halved, and if the percentage exceeds halfMark +/- deviation
		// X is recalculated to account for what is most likely a slanted Edge
		do {
			x += sampleSize;  // Moves the sample section to the Right by sampleSize
			
			for(int xScan = 0; xScan < sampleSize; xScan ++) {
			// Samples a column of the image from: [-sampleSize/2 + y] to [sampleSize/2 + y]
			for(int yScan = -sampleSize/2; yScan < sampleSize/2; yScan ++) {
				
				// Skip any parts outside of the Image
				if(yScan + y >= image.getHeight() || yScan + y < 0 || xScan + x < image.getWidth()) {
					sample[xScan][yScan + sampleSize/2] = false; 
					continue;
				}
													// Check if the "Green?" component of the color is > 0
				sample[xScan][yScan + sampleSize/2] = (image.getRGB(x + xScan, y + yScan) & 0xFF00) > 0;
				
				// The component is check if it is over 0 because the image is a Gray-Scale formated to represent
				// A Binary-Image were a value of 0 is "false" and a value of 1 is "true"
				
				// In a Gray-Scale Image all color components except Alpha are set to the same value
				
				// Color component part "0xFF00" is used to avoid the Alpha component
				// This is done because most common color formats use ARGB or RGBA
				// By selecting one of the two middle components the Alpha is completely avoided
			}}
			
			
			// TODO: Check for gaps in the sample, which can invalidating the percentage calculation  
			percentage = calcPercentage(sample);
		} while(Math.abs(percentage - halfMark) > deviation && percentage > 0);

		// ---------------------------------------------- ---------- ---------------------------------------------- \\
		// ---------------------------------------------- Y Tweaking ---------------------------------------------- \\
		
		// Slowly shifts back the checking region until it finds a valid section
		// Or the selection falls nearly completely back into the full shape
		while(Math.abs(percentage - halfMark) > deviation && percentage > halfMark - deviation) {
			x --;  // Slowly move the sample section to the Left by 1 pixel at a time
			
			for(int xScan = 0; xScan < sampleSize; xScan ++) {
			for(int yScan = -sampleSize/2; yScan < sampleSize/2; yScan ++) {
				
				// Skip any parts outside of the Image
				if(yScan + y >= image.getHeight() || yScan + y < 0 || xScan + x < image.getWidth()) {
					sample[xScan][yScan + sampleSize/2] = false; 
					continue;
				}
													// Check if the "Green?" component of the color is > 0
				sample[xScan][yScan + sampleSize/2] = (image.getRGB(x + xScan, y + yScan) & 0xFF00) > 0;
			}}
			
			// TODO: Check for gaps in the sample, which can invalidating the percentage calculation  
			percentage = calcPercentage(sample);
		}
	}
	
	private float calcPercentage(boolean[][] sample) {
		float total = 0.0f;
		for(boolean[] array : sample) {
			for(boolean bool : array) {
				total += bool ? 1 : 0;
			}
		}
		
		return total / (float)(sample.length * sample.length);				
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
