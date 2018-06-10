package dobotdll;
import com.sun.jna.Pointer;
import com.sun.jna.Structure;
import java.util.Arrays;
import java.util.List;
/**
 * Real-time pose<br>
 * <i>native declaration : line 5</i><br>
 * This file was autogenerated by <a href="http://jnaerator.googlecode.com/">JNAerator</a>,<br>
 * a tool written by <a href="http://ochafik.com/">Olivier Chafik</a> that <a href="http://code.google.com/p/jnaerator/wiki/CreditsAndLicense">uses a few opensource projects.</a>.<br>
 * For help, please visit <a href="http://nativelibs4java.googlecode.com/">NativeLibs4Java</a> , <a href="http://rococoa.dev.java.net/">Rococoa</a>, or <a href="http://jna.dev.java.net/">JNA</a>.
 */
public class Pose extends Structure {
	public float x;
	public float y;
	public float z;
	public float r;
	/** C type : float[4] */
	public float[] jointAngle = new float[4];
	public Pose() {
		super();
	}
	protected List<? > getFieldOrder() {
		return Arrays.asList("x", "y", "z", "r", "jointAngle");
	}
	/** @param jointAngle C type : float[4] */
	public Pose(float x, float y, float z, float r, float jointAngle[]) {
		super();
		this.x = x;
		this.y = y;
		this.z = z;
		this.r = r;
		if ((jointAngle.length != this.jointAngle.length)) 
			throw new IllegalArgumentException("Wrong array size !");
		this.jointAngle = jointAngle;
	}
	public Pose(Pointer peer) {
		super(peer);
	}
	public static class ByReference extends Pose implements Structure.ByReference {
		
	};
	public static class ByValue extends Pose implements Structure.ByValue {
		
	};
}