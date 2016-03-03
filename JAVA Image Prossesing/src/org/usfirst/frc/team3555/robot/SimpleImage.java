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
			SimpleBMPImage bmpImage = new SimpleBMPImage(buffer);
		    SimpleBinaryImage binaryImage = new SimpleBinaryImage(bmpImage.width, bmpImage.height);
			
			for(int y = 0; y < binaryImage.height; y ++) {
			for(int x = 0; x < binaryImage.width; x ++) {
				binaryImage.imageData[x + y * binaryImage.width] = bmpImage.imageData[x + y * bmpImage.width] != 255;
			}}
			
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
			
		    int padding = (info_Header.sizeImage / info_Header.height) - info_Header.width * 3;
	        if (padding == 4 || padding < 0)
	        	padding = 0;
			
			this.width = Math.abs(info_Header.width);
			this.height = Math.abs(info_Header.height);
			this.imageData = new int[width * height];
			
			byte[] readData = new byte[(width + padding) * 3 * height];
			buffer.get(readData, 0, readData.length);
			
			int i = 0;
			for(int y = 0; y < height; y ++) {
				for(int x = 0; x < width; x ++) {
					byte blue = readData[i ++];
					byte green = readData[i ++];
					byte red = readData[i ++];
					
					imageData[x + y * width] = red 	 << 24 & (0xFF << 24) | 
											   green << 16 & (0xFF << 16) | 
											   blue  <<  8 & (0xFF <<  8) |
											   255 ;
				}
				
				i += padding;
			}
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
			byte[] data = new byte[2 + 4 + 2 + 2 + 4];
			buffer.get(data, 0, data.length);
			int position = 0;
			
			header.type = File_IO.bytesToShort(data, position, true); 		position += 2;
			header.size = File_IO.bytesToInt(data, position, true); 		position += 4;
			header.reserved1 = File_IO.bytesToShort(data, position, true); 	position += 2;
			header.reserved2 = File_IO.bytesToShort(data, position, true); 	position += 2;
			header.offBit = File_IO.bytesToInt(data, position, true); 		position += 4;
			
			return header;
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
			byte[] data = new byte[4 + 4 + 4 + 2 + 2 + 4 + 4 + 4 + 4 + 4 + 4];
			buffer.get(data, 0, data.length);
			int position = 0;
			
			header.size = File_IO.bytesToInt(data, position, true); 		 position += 4;
			header.width = File_IO.bytesToInt(data, position, true); 		 position += 4;
			header.height = File_IO.bytesToInt(data, position, true); 		 position += 4;
			header.planes = File_IO.bytesToShort(data, position, true); 	 position += 2;
			header.bitCount = File_IO.bytesToShort(data, position, true); 	 position += 2;
			header.compression = File_IO.bytesToInt(data, position, true); 	 position += 4;
			header.sizeImage = File_IO.bytesToInt(data, position, true); 	 position += 4;
			header.xPelsPerMeter = File_IO.bytesToInt(data, position, true); position += 4; 
			header.yPelsPerMeter = File_IO.bytesToInt(data, position, true); position += 4; 
			header.clrUsed = File_IO.bytesToInt(data, position, true); 		 position += 4;
			header.clrImportant  = File_IO.bytesToInt(data, position, true); position += 4;
			
			return header;
		}
	}
}
