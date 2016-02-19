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
	
	
}
