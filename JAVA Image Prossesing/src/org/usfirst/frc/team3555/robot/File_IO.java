package org.usfirst.frc.team3555.robot;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.channels.FileChannel;

public class File_IO {
	public static boolean write(String path, boolean append, String... lines) {
		try {
			FileOutputStream out = new FileOutputStream(new File(path), append);
			
			for(String string : lines) {
				out.write(string.getBytes());
				out.flush();
			}
			
			out.close();
			return true;
		} catch(IOException e) {}
		
		return false;
	}
	
	public static boolean write(String path, ByteBuffer buffer) {		
		try {
			FileOutputStream out = new FileOutputStream(new File(path));
			FileChannel outChannel = out.getChannel();
			outChannel.write(buffer); out.close();
			return true;
		} catch(IOException e) {}
		
		return false;
	}
	
	public static ByteBuffer read(String path) {
		try {
			FileInputStream in = new FileInputStream(new File(path));
			FileChannel inChannel = in.getChannel();
			ByteBuffer buffer = ByteBuffer.allocate((int) inChannel.size());
			buffer.mark(); inChannel.read(buffer); in.close();
			
			return buffer;
		} catch(IOException e) {}
		
		return null;
	}
	
	public static int bytesToInt(byte[] bytes, int offset, boolean mostSignificantLast) {
		if(mostSignificantLast) {
			return (bytes[3 + offset] & (0xFF << 24)) << 24 
				 | (bytes[2 + offset] & (0xFF << 16)) << 16 
				 | (bytes[1 + offset] & (0xFF <<  8)) << 8 
				 | (bytes[0 + offset] & (0xFF <<  0)) << 0;
		} else {
			return (bytes[0 + offset] & (0xFF << 24)) << 24 
				 | (bytes[1 + offset] & (0xFF << 16)) << 16 
				 | (bytes[2 + offset] & (0xFF <<  8)) << 8 
				 | (bytes[3 + offset] & (0xFF <<  0)) << 0;
		}
	}
	
	public static short bytesToShort(byte[] bytes, int offset, boolean mostSignificantLast) {
		if(mostSignificantLast) {
			return (short) (
				  (bytes[1 + offset] & (0xFF << 8)) << 8 
				| (bytes[0 + offset] & (0xFF << 0)) << 0);
		} else {
			return (short) (
				   (bytes[0 + offset] & (0xFF << 8)) << 8 
				 | (bytes[1 + offset] & (0xFF << 0)) << 0);
		}
	}
}
