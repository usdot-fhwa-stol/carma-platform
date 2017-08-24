package gov.dot.fhwa.saxton.carma.route;

import com.esotericsoftware.yamlbeans.YamlException;
import com.esotericsoftware.yamlbeans.YamlReader;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.util.HashMap;
import java.util.List;

/**
 * The RouteWorker is responsible for implementing all ros agnostic logic of the Route package
 */
public class RouteWorker {

  protected HashMap<String, Route> availableRoutes = new HashMap<>();
  protected double crossTrackDistance;
  protected RouteSegment currentSegment;

  public RouteWorker(){}

  public RouteWorker(String database_path){
    File folder = new File(database_path);
    File[] listOfFiles = folder.listFiles();

    for (int i = 0; i < listOfFiles.length; i++) {
      if (listOfFiles[i].isFile()) {
        FileStrategy loadStrategy = new FileStrategy(listOfFiles[i].getPath());
        loadAdditionalRoute(loadStrategy);
      }
    }
  }
  public List<Route> handleGetAvailableRoutes(){
    return null;
  }

  public void handleSetActiveRoute(String routeID){

  }

  public void loadAdditionalRoute(IRouteLoadStrategy loadStrategy){
    Route route = loadStrategy.load();
    availableRoutes.put(route.getRouteID(), route);
  }

}
