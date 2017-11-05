package gov.dot.fhwa.saxton.carma.message;

public class HelperBSM {
	
	protected static final int MSG_COUNT_MIN = 0;
	protected static final int MSG_COUNT_MAX = 127;
	protected static final int ID_MIN = 0;
	protected static final int ID_MAX = 255;
	protected static final int DSECOND_MIN = 0;
	protected static final int DSECOND_MAX = 65535;
	
	protected static final int SECONDS_TO_MILLISECONDS = 1000;
	protected static final byte BRAKES_STATUS_UNAVAILABLE = 16;
	protected static final byte BRAKES_NOT_APPLIED = 0;
	protected static final byte BRAKES_APPLIED = 15;
	protected static final double YAWRATE_MAX = 327.67;
	protected static final double YAWRATE_MIN = -327.67;
	protected static final float VERTICAL_ACCELERATION_MIN = -24.696f;
	protected static final float VERTICAL_ACCELERATION_MAX = 24.892f;
	protected static final int ACCELERATION_MAX = 20;
	protected static final int ACCELERATION_MIN = -20;
	protected static final float VERTICAL_ACCELERATION_UNAVAILABLE = -24.892f;
	protected static final float ACCELERATION_UNAVAILABLE = 20.01f;
	protected static final int STEEL_WHEEL_ANGLE_MAX = 189;
	protected static final int STEEL_WHEEL_ANGLE_MIN = -189;
	protected static final float STEEL_WHEEL_ANGLE_UNAVAILABLE = 190.5f;
	protected static final float HEADING_MAX = 359.9875f;
	protected static final int HEADING_UNAVAILABLE = 360;
	protected static final float SPEED_MAX = 163.8f;
	protected static final float SPEED_UNAVAILABLE = 163.82f;
	protected static final double ACCURACY_ORIENTATION_MAX = 359.9945078786;
	protected static final float ACCURACY_MAX = 12.7f;
	protected static final float ELEVATION_MAX = 6143.9f;
	protected static final int LONGITUDE_MAX = 180;
	protected static final float ELEVATION_MIN = -409.5f;
	protected static final double LONGITUDE_MIN = -179.9999999;
	protected static final int LATITUDE_MAX = 90;
	protected static final int LATITUDE_MIN = -90;
	protected static final double ACCURACY_ORIENTATION_UNAVAILABLE = 65535 * 0.0054932479;
	protected static final float ACCURACY_UNAVAILABLE = 12.75f;
	protected static final float ELEVATION_UNAVAILABLE = -409.6f;
	protected static final double LONGITUDE_UNAVAILABLE = 180.0000001;
	protected static final double LATITUDE_UNAVAILABLE = 90.0000001;
	
	protected static final int ID_TIME_MAX = 3000;
	
	private int msgCnt = 0;
	private int id_0 = 0;
	private int id_1 = 0;
	private int id_2 = 0;
	private int id_3 = 0;
	private int secMark = 0;
	private int lat = 0;
	private int Lon = 0;
	private int elev = 0;
	private int accuracy_major = 0;
	private int accuracy_minor = 0;
	private int accuracy_orientation = 0;
	private int transmission = 0;
	private int speed = 0;
	private int heading = 0;
	private int angle = 0;
	private int acceleration_lon = 0;
	private int acceleration_lat = 0;
	private int acceleration_vert = 0;
	private int yaw_rate = 0;
	private int wheel_brakes_0 = 0;
	private int wheel_brakes_1 = 0;
	private int wheel_brakes_2 = 0;
	private int wheel_brakes_3 = 0;
	private int wheel_brakes_4 = 0;
	private int traction = 0;
	private int abs = 0;
	private int scs = 0;
	private int bba = 0;
	private int aux = 0;
	private int vehicle_width = 0;
	private int vehicle_length = 0;
}
