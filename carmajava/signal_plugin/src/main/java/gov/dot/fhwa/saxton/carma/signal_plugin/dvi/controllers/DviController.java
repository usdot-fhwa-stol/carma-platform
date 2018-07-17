package gov.dot.fhwa.saxton.glidepath.dvi.controllers;


import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicLong;

import gov.dot.fhwa.saxton.glidepath.IConsumerTask;
import gov.dot.fhwa.saxton.glidepath.dvi.GlidepathAppConfig;
import gov.dot.fhwa.saxton.glidepath.dvi.services.ConsumerFactory;
import gov.dot.fhwa.saxton.glidepath.dvi.services.DviExecutorService;
import gov.dot.fhwa.saxton.glidepath.logger.ILogger;
import gov.dot.fhwa.saxton.glidepath.logger.LoggerManager;
import gov.dot.fhwa.saxton.glidepath.dvi.domain.*;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.web.bind.annotation.RequestMapping;
import org.springframework.web.bind.annotation.RequestParam;
import org.springframework.web.bind.annotation.RestController;

import javax.inject.Inject;

@RestController
public class DviController {

    @Inject
    GlidepathAppConfig appConfig;

    @Autowired
    DviExecutorService dviService;

    private static ILogger logger = LoggerManager.getLogger(DviController.class);

    private final AtomicLong counter = new AtomicLong();

    @RequestMapping("/initialize")
    public AjaxResponse initialize(@RequestParam(value="device", required=false, defaultValue="all") String name) {

        boolean bResult = dviService.start();
        return new AjaxResponse(bResult, "The DviExecutorService has started...");
    }

    @RequestMapping("/terminate")
    public AjaxResponse terminate(@RequestParam(value="device", required=false, defaultValue="all") String name) {
        dviService.stop();
        return new AjaxResponse(true, "The DviExecutorService has stopped.");
    }


    @RequestMapping("/setDviParameters")
    public OperatingSpeedResponse setOperatingSpeed(@RequestParam(value="operatingSpeed", required=true) String operatingSpeed,
                                                    @RequestParam(value="logFilePrefix", required=true) String logFilePrefix)   {

        // this is the beginning of starting eco drive
        // if the user clicked EcoDrive GO button without clicking Record Data, start recording data now to ensure
        //   we are logging
        if (!dviService.getRecordData())   {
            dviService.setRecordData(true);
        }

        String statusMessage =  "Setting operating speed: " + operatingSpeed;
        logger.info(ILogger.TAG_DVI, statusMessage);
        logger.info(ILogger.TAG_DVI, "Setting prefix for log files to: " + logFilePrefix);

        DviParameters dviParameters = DviParameters.getInstance();

        double doubleOperatingSpeed = Double.parseDouble(operatingSpeed);
        dviParameters.setOperatingSpeed(doubleOperatingSpeed);

        dviParameters.setLogFileNames(logFilePrefix);

        boolean bStateChange = GlidepathStateModel.getInstance().changeState(GlidepathState.ENGAGED, null);

        return new OperatingSpeedResponse(true, statusMessage, operatingSpeed);
    }


    @RequestMapping("/setEcoDrive")
    public AjaxResponse setEcoDrive(@RequestParam(value="ecoDrive", required=true) boolean ecoDrive) {
        boolean bResult = false;

        if (!ecoDrive)   {
            bResult = GlidepathStateModel.getInstance().changeState(GlidepathState.DISENGAGED, null);
        }
        else   {
            bResult = GlidepathStateModel.getInstance().changeState(GlidepathState.ENGAGED, null);
        }

        String statusMessage =  "Setting ecoDrive: " + ecoDrive;
        logger.info(ILogger.TAG_DVI, statusMessage);

        return new AjaxResponse(true, statusMessage);
    }

    @RequestMapping("/ajaxStringList")
    public ConsumerListResponse ajaxAction(@RequestParam(value="action", required=true) String action) {
        boolean bResult = false;

        List<String> consumers = null;

        switch (action)   {
            case "listConsumers":
                consumers = ConsumerFactory.getConsumersClassNames();
                break;

            default:
                consumers = new ArrayList<String>();
                break;
        }

        String statusMessage =  "Retrieved Consumer class names: " + consumers.size();
        logger.info(ILogger.TAG_DVI, statusMessage);

        return new ConsumerListResponse(true, statusMessage, consumers);
    }


    @RequestMapping("/ajaxAddConsumers")
    public AjaxResponse ajaxAddConsumer(@RequestParam(value="consumers[]", required=true) String[] consumers ) {
        boolean bResult = false;

        List<IConsumerTask> consumerList = new ArrayList<IConsumerTask>();

        for (String consumer : consumers)   {
            IConsumerTask consumerTask = ConsumerFactory.newConsumer(consumer);
            consumerList.add(consumerTask);
        }

        dviService.setConsumers(consumerList);

        String statusMessage =  "Added Consumers: " + consumerList.size();
        logger.info(ILogger.TAG_DVI,statusMessage);

        return new AjaxResponse(true, statusMessage);
    }

    @RequestMapping("/setLogging")
    public AjaxResponse setLogging(@RequestParam(value="logging", required=true) boolean flag) {
        boolean bResult = false;

        //  stop logging, shutdown executor, move logs and respond to ajax request
        if (!flag)   {
            dviService.autoRollLogs();
        }
        // start logging
        else   {
            dviService.setRecordData(true);
        }

        String statusMessage =  "Setting logging on/off: " + flag;
        logger.info(ILogger.TAG_DVI, statusMessage);

        return new AjaxResponse(true, statusMessage);
    }

}