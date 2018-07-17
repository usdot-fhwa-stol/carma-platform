package gov.dot.fhwa.saxton.glidepath.appcommon;

public class Constants {

	// for intersections geometry
	public static final int				THRESHOLD_DIST				= 600;		//used for various distance comparisons, cm (a little wider than a typical lane)
	public static final double			RADIUS_OF_EARTH_METERS		= 6371009.0;//IUGG mean radius (http://en.wikipedia.org/wiki/Earth_radius)
																				//UCR had used 6378137, which is equatorial radius
	public static final double			PI 							= 3.141592653589793238462;


    // Constants for JAUS initialization with xgv
    public static final int            JAUS_INBOUND_UDP_PORT       = 3794;
    public static final int            XGV_HANDLER_JAUS_ID         = 125941519; // Letter-number cipher for leidos
    public static final int            JAUS_MAX_PACKET_SIZE        = 1000;
    
    //other communication parameters
    public static final int				ASD_MAX_PACKET_SIZE			= 1472;

    // Conversion Factor Constants
    public static final double			MPS_TO_MPH                  = 2.2369362920544;
    public static final double			METERS_TO_FEET              = 3.2808;

    public static final double          MS_TO_SEC                   = 0.001;

    public static final String          LOG_FILE                    = "logs/speedcontrol.log";
}
