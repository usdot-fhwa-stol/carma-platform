package gov.dot.fhwa.saxton.glidepath.filter;


/**
 * Creates a concrete ISmoother class based on the provided class name
 * 
 * @author starkj
 *
 */
public class DataFilterFactory {

	public static IDataFilter newInstance(String className) {
		@SuppressWarnings("rawtypes")
		Class tClass = null;
		
        try   {
            tClass = Class.forName(className);
        }
        catch(ClassNotFoundException cnfe)   {
            tClass = NoFilter.class;
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

        return (IDataFilter)newObject;
	}
}
