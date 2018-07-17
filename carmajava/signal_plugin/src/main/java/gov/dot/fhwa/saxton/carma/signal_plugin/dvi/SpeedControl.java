package gov.dot.fhwa.saxton.glidepath.dvi;

import gov.dot.fhwa.saxton.glidepath.appcommon.utils.GlidepathApplicationContext;
import gov.dot.fhwa.saxton.glidepath.dvi.services.DviExecutorService;
import gov.dot.fhwa.saxton.glidepath.logger.ILogger;
import gov.dot.fhwa.saxton.glidepath.logger.LoggerManager;
import org.springframework.boot.SpringApplication;
import org.springframework.boot.autoconfigure.EnableAutoConfiguration;
import org.springframework.context.ConfigurableApplicationContext;
import org.springframework.context.annotation.ComponentScan;

import java.io.*;
import java.util.Scanner;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

@ComponentScan
@EnableAutoConfiguration
public class SpeedControl {

    public static void main(String[] args) {
        /*
         * Initalize the logger type before we do anything else
         *
         * We have to do this manually since once we load up Spring (which normally loads config files for us) it's too
         * late, loggers have already been initialized. So we have to read our config files ourselves and regex out the
         * entry we want.
         */
        boolean useLog4j2 = false;
        File configOverride = new File("dvi.properties");
        if (configOverride.exists()) {
            try (FileInputStream fis = new FileInputStream(configOverride)) {
                useLog4j2 = findUseLog4J(fis);
            } catch (FileNotFoundException e) {
                e.printStackTrace();
            } catch (IOException e) {
                e.printStackTrace();
            }
        } else {
            InputStream jarredConfigFileStream = SpeedControl.class.getClassLoader().getResourceAsStream("dvi.properties");
            useLog4j2 = findUseLog4J(jarredConfigFileStream);
        }
        LoggerManager.useLog4J2(useLog4j2);


        ConfigurableApplicationContext context = SpringApplication.run(SpeedControl.class, args);

        // set our context for non spring managed classes
        GlidepathApplicationContext.getInstance().setApplicationContext(context);
        GlidepathAppConfig config = ((GlidepathAppConfig) GlidepathApplicationContext.getInstance().getAppConfig());

        //SimulatedDviExecutorService service = context.getBean(SimulatedDviExecutorService.class);
        DviExecutorService service = context.getBean(DviExecutorService.class);

        LoggerManager.setRealTimeOutput(config.getLogRealTimeOutput());
        ILogger logger = LoggerManager.getLogger(SpeedControl.class);

        logger.infof("TAG", "####### SpeedControl started ########");
        try   {
            LoggerManager.writeToDisk();
        }
        catch(IOException ioe)   {
            System.err.println("Error writing log to disk: " + ioe.getMessage());
        }

        if (service.getAutoStartConsumption())   {
            service.start();
        }

    }

    /**
     * Scans an input stream for a config line containing information on logger selection
     * @param is The input stream to be scanned
     * @return true, if a value of true is found. false otherwise.
     */
    private static boolean findUseLog4J(InputStream is) {
        Scanner s = new Scanner(is);

        /*
         * Match any amount of whitespace, followed by "log.uselog4j2", then any amount of whitespace, followed by "=",
         * followed by any amount of whitespace, followed by "true" or "false", then any amount of whitespace or a
         * comment to end the line.
         */
        Pattern regex = Pattern.compile("^\\s*log\\.uselog4j2\\s*=\\s*(true|false)\\s*(#.*)?$");

        try {
            while (s.hasNextLine()) {
                String line = s.next();

                Matcher m = regex.matcher(line);
                if (m.matches()) {
                    return Boolean.parseBoolean(m.group(1));
                }
            }
        }catch (Exception e) {
            //allow the method to end with a failure
        }

        return false;
    }
}