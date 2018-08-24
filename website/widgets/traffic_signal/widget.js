/***
* Create a unique namespace for each plugin widget to minimize collision of same name variables or functions.

TODO: Update to traffic_signal and remove refrences to lanechange as temporary
***/

CarmaJS.registerNamespace("CarmaJS.WidgetFramework.TrafficSignal");
//CarmaJS.registerNamespace("CarmaJS.WidgetFramework.LaneChange");

CarmaJS.WidgetFramework.TrafficSignal = (function () {
//CarmaJS.WidgetFramework.LaneChange = (function () {

        //*** Private Variables ***
        var svgLayer;
        var svgLayerSpeed;

        //Listeners
        var listenerSpeedAccel;
        var listenerCANSpeed;

        //*** Widget Install Folder ***
        //Currently the URL path from document or window are pointing to the page, not the actual folder location.
        //Therefore this needs to be hardcoded.
        //TODO: However, this could be set by widgetfw based on final install folder naming convention.
        var installfoldername = 'widgets/traffic_signal/';
        //var installfoldername = 'widgets/lane_change/';

        //*** Functions ***

        /***
        * Display the Speed Limit
        ***/
        var checkRouteState = function () {

            //Get Route State
            listenerRouteState = new ROSLIB.Topic({
                ros: ros,
                name: t_route_state,
                messageType: 'cav_msgs/RouteState'
            });

            listenerRouteState.subscribe(function (message) {

                if (svgLayerSpeed == null)
                    return;

                if (svgLayerSpeed.getElementById("svgLimitText") == null)
                    return;

                if (message.current_segment.waypoint.speed_limit != null && message.current_segment.waypoint.speed_limit != 'undefined')
                {
                    var speed_limit = String(message.current_segment.waypoint.speed_limit);

                    svgLayerSpeed.getElementById("svgLimitText").innerHTML = speed_limit.padStart(2,'0');
                }
            });
        };

        /***
        *    Display commanded speed
        ***/
        var showSpeedAccelInfo = function () {

            listenerSpeedAccel = new ROSLIB.Topic({
                ros: ros,
                name: t_cmd_speed,
                messageType: 'cav_msgs/SpeedAccel'
            });

            listenerSpeedAccel.subscribe(function (message) {

                if (svgLayerSpeed == null)
                    return;

                if (svgLayerSpeed.getElementById('svgCmdText') == null)
                    return;

                if (message.speed != null && message.speed != 'undefined')
                {
                    var speedCmd = String(Math.round(message.speed * METER_TO_MPH));

                    svgLayerSpeed.getElementById("svgCmdText").innerHTML = speedCmd.padStart(2,'0');
                }

            });
        };

        /*
            Display the actual speed
        */
        var showCANSpeeds = function () {

            listenerCANSpeed = new ROSLIB.Topic({
                ros: ros,
                name: t_can_speed,
                messageType: 'std_msgs/Float64'
            });

            listenerCANSpeed.subscribe(function (message) {

                if (svgLayerSpeed == null)
                    return;

                if (svgLayerSpeed.getElementById('svgActualText') == null)
                    return;

                if (message.data != null && message.data != 'undefined')
                {
                    var speedActual = String(Math.round(message.data * METER_TO_MPH));

                    svgLayerSpeed.getElementById("svgActualText").innerHTML = speedActual.padStart(2,'0');
                }


            });
        };

        /*
            Display the intersection info
            //TODO: Implement
        */
        var showIntersectionInfo = function () {

            listenerIntersections = new ROSLIB.Topic({
                ros: ros,
                name: t_intersections,
                messageType: 'std_msgs/Float64'
            });

            listenerIntersections.subscribe(function (message) {

                if (svgLayer == null)
                    return;

                if (svgLayer.getElementById("svgCircleTopText") == null)
                    return;

                svgLayer.getElementById("svgCircleTopText").innerHTML = "";
            });
        };

        /***
        * Custom widgets using JQuery Widget Framework.
        * NOTE: that widget framework namespace can only be one level deep.
        ***/
        //$.widget("CarmaJS.trafficSignalIntersection", {
        $.widget("CarmaJS.trafficSignalIntersection", {
           _create: function() {
                var myDiv = $("<object id='tsp_speed' type='image/svg+xml' data='" + installfoldername + "tsp_speed.svg' style='width: 325px; height: 500px;'></object>"
                          + "<object id='tsp_intx1' type='image/svg+xml' data='" + installfoldername + "tsp_signal_intx.svg' style='width: 325px; height: 500px;'></object>"
                          + "<object id='tsp_intx2' type='image/svg+xml' data='" + installfoldername + "tsp_signal_intx.svg' style='width: 325px; height: 500px;'></object>"
                          + "<object id='tsp_intx3' type='image/svg+xml' data='" + installfoldername + "tsp_signal_intx.svg' style='width: 325px; height: 500px;'></object>");

                $(this.element).append(myDiv);

                var tsp_speed = document.getElementById("tsp_speed");

                tsp_speed.addEventListener("load", function(){
                    var svgDocSpeed = tsp_speed.contentDocument;
                    //alert("svgDocIntx1: " + svgDocIntx1);

                    svgLayerSpeed = svgDocSpeed.getElementById("svgLayerSpeed");
                    //alert("svgLayerSpeed0: " + svgLayerSpeed);

                     if (svgLayerSpeed != null) {
                            //Reset
                            svgLayerSpeed.getElementById("svgLimitText").innerHTML = "00";
                            svgLayerSpeed.getElementById("svgCmdText").innerHTML = "00";
                            svgLayerSpeed.getElementById("svgActualText").innerHTML = "00";
                     }
                });

                var svgIntx1 = document.getElementById("tsp_intx1");

                svgIntx1.addEventListener("load", function () {
                    //alert('here');

                    //var svgDocIntx1 = svgIntx1.getSVGDocument();
                    var svgDocIntx1 = svgIntx1.contentDocument;
                    //alert("svgDocIntx1: " + svgDocIntx1);

                    svgLayer = svgDocIntx1.getElementById("svgLayer");
                    //alert("svgLayer0: " + svgLayer);

                    if (svgLayer != null) {
                        //SIGNAL ON
                        //Top Signal - Red
                        svgLayer.getElementById("svgCircleTopText").innerHTML = "XX";
                        svgLayer.getElementById("svgCircleTop").setAttribute("style", "fill:#EF2E65;");
                        svgLayer.getElementById("svgCircleTopPath").setAttribute("style", "fill:#CC0E57;");

                        //Middle Signal - Yellow
                        svgLayer.getElementById("svgCircleMiddleText").innerHTML = "XX";
                        svgLayer.getElementById("svgCircleMiddle").setAttribute("style", "fill:#EDBF2F;");
                        svgLayer.getElementById("svgCircleMiddlePath").setAttribute("style", "fill:#DBA81B;");

                        //Bottom Signal - Green
                        svgLayer.getElementById("svgCircleBottomText").innerHTML = "XX";
                        svgLayer.getElementById("svgCircleBottom").setAttribute("style", "fill:#32D85D;");
                        svgLayer.getElementById("svgCircleBottomPath").setAttribute("style", "fill:#19BF55;");
                    }
                });
                //TRIAL1: END
           },
           _destroy: function() {
                //TODO:
                if (listenerRouteState)
                    listenerRouteState.unsubscribe();
                if (listenerSpeedAccel)
                    listenerSpeedAccel.unsubscribe();
                if (listenerCANSpeed)
                   listenerCANSpeed.unsubscribe();
                //if (listenerIntersections) //TODO:
                //   listenerIntersections.unsubscribe();
                this.element.empty();
                this._super();
           },
           checkRouteState: function(){
              checkRouteState();
           },
           showSpeedAccelInfo: function(){
              showSpeedAccelInfo();
           },
           showCANSpeeds: function(){
              showCANSpeeds();
           },
           showIntersectionInfo: function(){
              showIntersectionInfo();
           }
        });

        /*** Public Functions ***
        This is the public function call to setup the different widgets available in this plugin.
        ***/
        var loadCustomWidget = function(container) {

            //Load the signal
            container.trafficSignalIntersection();
            container.trafficSignalIntersection("checkRouteState", null);
            container.trafficSignalIntersection("showSpeedAccelInfo", null);
            container.trafficSignalIntersection("showCANSpeeds", null);
            //container.trafficSignalIntersection("showIntersectionInfo", null); //TODO:
        };

        //*** Public API  ***
        return {
            loadCustomWidget: loadCustomWidget
        };

})(); //CarmaJS.WidgetFramework.Cruising