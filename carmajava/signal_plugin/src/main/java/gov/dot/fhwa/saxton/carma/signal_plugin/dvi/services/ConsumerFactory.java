package gov.dot.fhwa.saxton.carma.signal_plugin.dvi.services;


import gov.dot.fhwa.saxton.carma.signal_plugin.IConsumerTask;
import gov.dot.fhwa.saxton.carma.signal_plugin.appcommon.utils.GlidepathApplicationContext;
import gov.dot.fhwa.saxton.carma.signal_plugin.appcommon.GlidepathAppConfig;

import java.util.ArrayList;
import java.util.List;

public class ConsumerFactory {


    private static List<IConsumerTask> consumers = new ArrayList<IConsumerTask>();
    private static List<String> consumerClasses = new ArrayList<String>();

    public static List<IConsumerTask> getConsumers()   {
        if (consumers.isEmpty())   {
            loadConsumers();
        }

        return consumers;
    }


    public static List<String> getConsumersClassNames()   {
        if (consumers.isEmpty())   {
            loadConsumers();
        }

        return consumerClasses;
    }


    /**
     * Return consumer based on provided class name as String
     *
     * @param fullClassName
     * @return
     */
    public static IConsumerTask getConsumer(String fullClassName)   {
        IConsumerTask consumer = null;

        if (consumers.isEmpty())   {
            loadConsumers();
        }

        for (IConsumerTask consumerTask : consumers)   {
            String thisClassName = consumerTask.getClass().getName();
            if (thisClassName.equals(fullClassName))   {
                return consumerTask;
            }
        }

        return consumer;
    }


    /**
     * Iterate configuration and look for consumer.# class names.
     *
     * We will load configured consumer class names, then iterate the list and create each consumer using
     * reflection.
     *
     * The class names of each consumer is available via web interface via consumers.html
     *
     */
    protected static void loadConsumers()   {
        GlidepathAppConfig appConfig = ((GlidepathAppConfig) GlidepathApplicationContext.getInstance().getAppConfig());

        String consumerClazz = "";
        int count = 0;

        // iterate configuration to get consumer classes, must be consecutive .# as we stop on first missing #
        while (consumerClazz != null)    {
            String property = "consumer." + String.format("%d", count);
            consumerClazz = appConfig.getProperty(property);

            if (consumerClazz != null)   {
                consumerClasses.add(consumerClazz);
            }

            count += 1;
        }

        // iterate configured consumers and instantiate and add to consumers list
        for (String consumerClass : consumerClasses)   {
            IConsumerTask newObject = newConsumer(consumerClass);
            consumers.add((IConsumerTask) newObject);
        }

    }


    /**
     * Reflection helper to create a consumer object based on class name
     *
     * @param consumerClassName
     * @return
     */
    public static IConsumerTask newConsumer(String consumerClassName)   {
        Class tClass = null;

        try   {
            tClass = Class.forName(consumerClassName);
        }
        catch(ClassNotFoundException cnfe)   {

        }

        Object newObject = null;
        try   {
            newObject = tClass.newInstance();
        }
        catch (InstantiationException e) {
            e.printStackTrace();
        } catch (IllegalAccessException e) {
            e.printStackTrace();
        }

        if (newObject instanceof IConsumerTask)   {
            return (IConsumerTask) newObject;
        }

        return (IConsumerTask) newObject;
    }

}
