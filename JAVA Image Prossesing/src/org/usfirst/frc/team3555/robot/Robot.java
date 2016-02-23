package org.usfirst.frc.team3555.robot;

import static com.ni.vision.NIVision.RGB_BLACK;
import static com.ni.vision.NIVision.imaqCountParticles;
import static com.ni.vision.NIVision.imaqCreateImage;
import static com.ni.vision.NIVision.imaqDrawLineOnImage;
import static com.ni.vision.NIVision.imaqDuplicate;
import static com.ni.vision.NIVision.imaqMask;
import static com.ni.vision.NIVision.imaqSetSimpleCalibration;
import static com.ni.vision.NIVision.imaqWriteBMPFile;

import java.nio.ByteBuffer;

import org.usfirst.frc.team3555.robot.ShapeMatcher.AABB;
import org.usfirst.frc.team3555.robot.ShapeMatcher.Sample;
import org.usfirst.frc.team3555.robot.SimpleImage.SimpleBinaryImage;

import com.ni.vision.NIVision.AxisOrientation;
import com.ni.vision.NIVision.CalibrationUnit;
import com.ni.vision.NIVision.CoordinateSystem;
import com.ni.vision.NIVision.DrawMode;
import com.ni.vision.NIVision.GridDescriptor;
import com.ni.vision.NIVision.Image;
import com.ni.vision.NIVision.ImageType;
import com.ni.vision.NIVision.Point;
import com.ni.vision.NIVision.PointFloat;
import com.ni.vision.NIVision.ScalingMethod;

import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.vision.USBCamera;

public class Robot extends SampleRobot {
	private USBCamera camera;
	private Joystick joyOp;
	
	public Robot() {
		joyOp = new Joystick(1);
		camera = new USBCamera("cam1");
		camera.startCapture();
	}

	public void operatorControl() {
		int lastImage = 0;
		camera.setSize(320, 240); 
		
		while (isOperatorControl() && isEnabled()) {
			// Acquire Image
			Image maskImage = imaqCreateImage(ImageType.IMAGE_U8, 7);			
			Image image = imaqCreateImage(ImageType.IMAGE_U8, 7);
			camera.getImage(image);
			imaqDuplicate(maskImage, image);
			
			// Filter Image
			Image filterImage = imaqCreateImage(ImageType.IMAGE_U8, 7);
			filterImage = ImageFilter.filter(filterImage, image);
			imaqMask(maskImage, maskImage, filterImage);
			
			// "Calibrate" Image
			GridDescriptor gridDescriptor = new GridDescriptor(1, 1, CalibrationUnit.UNDEFINED);
			CoordinateSystem coordinateSystem = new CoordinateSystem(new PointFloat(0, 0), 0, AxisOrientation.INDIRECT);
			imaqSetSimpleCalibration(filterImage, ScalingMethod.SCALE_TO_FIT, 1, gridDescriptor, coordinateSystem);
			
			// Setup AABB for all particles
			int particleCount = imaqCountParticles(filterImage, 1);
			SmartDashboard.putNumber("Number of Particles: ", particleCount);
			AABB[] particles = new AABB[particleCount];
			for(int i = 0; i < particleCount; i ++) {
				particles[i] = AABB.loadAABB(filterImage, true, i);
			}
			
			// Save && Reload the Image - Converts from ImaqImage to SimpleBinaryImage
			long startTime = System.currentTimeMillis();
			imaqWriteBMPFile(filterImage, "/passThrough1.bmp", 0, RGB_BLACK);
			ByteBuffer readImage = File_IO.read("/passThrough.bmp"); readImage.reset();
			SimpleBinaryImage simpleImage = SimpleBinaryImage.loadBMPImage(readImage);
			SmartDashboard.putNumber("Time: ", System.currentTimeMillis() - startTime);
			
			// For every AABB (a.k.a Every Particle)
			for(AABB aabb : particles) { // Shape Match
				Sample[] corner = ShapeMatcher.findBoundingBoxCorner(aabb, simpleImage, 16);
				
//				for(int i = 0; i < corner.length; i ++) { // Draw line connecting from each "Corner"
//					imaqDrawLineOnImage(filterImage, filterImage, DrawMode.DRAW_VALUE, 
//							new Point(corner[i].getCenterX(), corner[i].getCenterY()), 
//							new Point(corner[(i + 1) % corner.length].getCenterX(), 
//									corner[(i + 1) % corner.length].getCenterY()), 255);
//				}
			}
			
			// Select which Image to Draw
			boolean newImageUse = false;
			for(int i = 1; i < 12; i ++) {
				if(i == 4) continue;
				if(!joyOp.getRawButton(i)) continue;
				
				if(i == 1) { CameraServer.getInstance().setImage(image); lastImage = i; newImageUse = true;}
				if(i == 10) { CameraServer.getInstance().setImage(maskImage); lastImage = i; newImageUse = true;}
				if(i == 11) { CameraServer.getInstance().setImage(filterImage); lastImage = i; newImageUse = true;}
			}
			
			SmartDashboard.putNumber("Display Image: ", lastImage);
			
			// Holds the Image on the screen
			if(!newImageUse) {
				if(lastImage == 1) { CameraServer.getInstance().setImage(image);}
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
			filterImage.free();
		}
	}
}