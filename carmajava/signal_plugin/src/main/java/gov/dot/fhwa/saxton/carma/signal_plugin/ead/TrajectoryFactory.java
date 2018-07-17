package gov.dot.fhwa.saxton.carma.signal_plugin.ead;

/**
 * Factory method to instantiate an appropriate ITrajectory concrete class
 */
public class TrajectoryFactory {

    /**
     * Reflection helper to create a ITrajectory object based on class name
     *
     * @param trajectoryClassName
     * @return
     */
    public static ITrajectory newInstance(String trajectoryClassName)   {
        @SuppressWarnings("rawtypes")
		Class tClass = null;

        try   {
            tClass = Class.forName(trajectoryClassName);
        }
        catch(ClassNotFoundException cnfe)   {
            tClass = SimulatedTrajectory.class;
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

        return (ITrajectory) newObject;
    }

}
