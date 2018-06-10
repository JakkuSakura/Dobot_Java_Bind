package dobotdll;
import com.sun.jna.Pointer;
import com.sun.jna.Structure;
import java.util.Arrays;
import java.util.List;
/**
 * JOG related<br>
 * <i>native declaration : line 68</i><br>
 * This file was autogenerated by <a href="http://jnaerator.googlecode.com/">JNAerator</a>,<br>
 * a tool written by <a href="http://ochafik.com/">Olivier Chafik</a> that <a href="http://code.google.com/p/jnaerator/wiki/CreditsAndLicense">uses a few opensource projects.</a>.<br>
 * For help, please visit <a href="http://nativelibs4java.googlecode.com/">NativeLibs4Java</a> , <a href="http://rococoa.dev.java.net/">Rococoa</a>, or <a href="http://jna.dev.java.net/">JNA</a>.
 */
public class JOGJointParams extends Structure {
	/** C type : float[4] */
	public float[] velocity = new float[4];
	/** C type : float[4] */
	public float[] acceleration = new float[4];
	public JOGJointParams() {
		super();
	}
	protected List<? > getFieldOrder() {
		return Arrays.asList("velocity", "acceleration");
	}
	/**
	 * @param velocity C type : float[4]<br>
	 * @param acceleration C type : float[4]
	 */
	public JOGJointParams(float velocity[], float acceleration[]) {
		super();
		if ((velocity.length != this.velocity.length)) 
			throw new IllegalArgumentException("Wrong array size !");
		this.velocity = velocity;
		if ((acceleration.length != this.acceleration.length)) 
			throw new IllegalArgumentException("Wrong array size !");
		this.acceleration = acceleration;
	}
	public JOGJointParams(Pointer peer) {
		super(peer);
	}
	public static class ByReference extends JOGJointParams implements Structure.ByReference {
		
	};
	public static class ByValue extends JOGJointParams implements Structure.ByValue {
		
	};
}
