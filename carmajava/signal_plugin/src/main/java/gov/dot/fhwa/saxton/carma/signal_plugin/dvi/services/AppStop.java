package gov.dot.fhwa.saxton.carma.signal_plugin.dvi.services;

import gov.dot.fhwa.saxton.carma.signal_plugin.appcommon.utils.GlidepathApplicationContext;
import org.springframework.context.ConfigurableApplicationContext;

/**
 * A Runnable class to stop the application
 *
 */
public class AppStop implements Runnable {

    @Override
    public void run() {
        ConfigurableApplicationContext springApp = (ConfigurableApplicationContext) GlidepathApplicationContext.getInstance().getApplicationContext();
        springApp.close();
        System.exit(0);
    }

}
