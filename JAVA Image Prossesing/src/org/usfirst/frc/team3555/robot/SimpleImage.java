package org.usfirst.frc.team3555.robot;

import java.nio.ByteBuffer;

public class SimpleImage {
	public static class SimpleBinaryImage {
		private int width, height;
		private boolean[] imageData;
		
		private SimpleBinaryImage(int width, int height) {
			this.width = width;
			this.height = height;
			
			imageData = new boolean[width * height];
		}
		
		public static SimpleBinaryImage loadBMPImage(ByteBuffer buffer) {
			BMP_Header.read(buffer);
			BMP_Info_Header info_Header = BMP_Info_Header.read(buffer);
		    int padding = (4 - (info_Header.width * 3) % 4) % 4;
			
		    SimpleBinaryImage binaryImage = new SimpleBinaryImage(info_Header.width, Math.abs(info_Header.height));
			
			for(int y = 0; y < binaryImage.height; y ++) {
				for(int x = 0; x < binaryImage.width; x ++) {
					RGB_Triple rgb = RGB_Triple.read(buffer);
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
		
		public int getWidth() { return width; }
		public int getHeight() { return height; }
		
		public int getRGB(int x, int y) {
			return imageData[x + y * width];
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
}
