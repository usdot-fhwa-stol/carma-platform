package gov.dot.fhwa.saxton.glidepath.ead;

/**
 * Creates a concrete IEad class based on provided class name.
 */
public class EadFactory {
	
	public static IEad newInstance(String className) {
		@SuppressWarnings("rawtypes")
		Class tClass;
		
        try   {
            tClass = Class.forName(className);
        }
        catch(ClassNotFoundException cnfe)   {
            tClass = EadAStar.class;
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

        return (IEad)newObject;
	}
	
}
