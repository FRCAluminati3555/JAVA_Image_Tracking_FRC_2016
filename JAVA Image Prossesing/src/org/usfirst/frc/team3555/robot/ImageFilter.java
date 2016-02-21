package org.usfirst.frc.team3555.robot;

import static com.ni.vision.NIVision.imaqAnd;
import static com.ni.vision.NIVision.imaqConvexHull;
import static com.ni.vision.NIVision.imaqCreateImage;
import static com.ni.vision.NIVision.imaqExtractColorPlanes;
import static com.ni.vision.NIVision.imaqNand;
import static com.ni.vision.NIVision.imaqOr;
import static com.ni.vision.NIVision.imaqParticleFilter4;
import static com.ni.vision.NIVision.imaqRejectBorder;
import static com.ni.vision.NIVision.imaqThreshold;

import com.ni.vision.NIVision.ColorMode;
import com.ni.vision.NIVision.Image;
import com.ni.vision.NIVision.ImageType;
import com.ni.vision.NIVision.MeasurementType;
import com.ni.vision.NIVision.ParticleFilterCriteria2;
import com.ni.vision.NIVision.ParticleFilterOptions2;

public class ImageFilter {
	public static Image filter(Image dest, Image src) {
		if(dest == null)
			dest = imaqCreateImage(ImageType.IMAGE_U8, 7);
		
		Image compositThreshold = imaqCreateImage(ImageType.IMAGE_U8, 7);
		thresholdProsses(compositThreshold, src);
		
		Image filterImage = imaqCreateImage(ImageType.IMAGE_U8, 7);
		removeSmallParicles(filterImage, compositThreshold, 150);
		imaqRejectBorder(filterImage, filterImage, 1);
		imaqConvexHull(filterImage, filterImage, 1);
		removeElogatedParticles(dest, filterImage, 3);
		
		compositThreshold.free();
		filterImage.free();
		
		return dest;
	}
	
	private static Image thresholdProsses(Image dest, Image source) {
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

	public static Image removeSmallParicles(Image dest, Image source, int minParimeter) {
		ParticleFilterCriteria2[] particleCriteria = new ParticleFilterCriteria2[] { new ParticleFilterCriteria2(
				MeasurementType.MT_PERIMETER, 0, minParimeter, 0, 0) };
		
		ParticleFilterOptions2 particleFilterOptions = new ParticleFilterOptions2(1, 0, 0, 1);
		imaqParticleFilter4(dest, source, particleCriteria, particleFilterOptions, null);
		particleFilterOptions.free();
		
		return dest;
	}
	
	public static Image removeElogatedParticles(Image dest, Image source, int maxEloncation) {
		ParticleFilterCriteria2[] particleCriteria = new ParticleFilterCriteria2[] { new ParticleFilterCriteria2(
				MeasurementType.MT_ELONGATION_FACTOR, maxEloncation, 65536, 0, 0) };
		
		ParticleFilterOptions2 particleFilterOptions = new ParticleFilterOptions2(1, 0, 0, 1);
		imaqParticleFilter4(dest, source, particleCriteria, particleFilterOptions, null);
		particleFilterOptions.free();
		
		return dest;
	}
}
