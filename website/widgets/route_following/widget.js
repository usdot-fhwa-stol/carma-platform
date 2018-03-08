/***
* Create a unique namespace for each plugin widget to minimize collision of same name variables or functions.
***/
CarmaJS.registerNamespace("CarmaJS.WidgetFramework.RouteFollowing");

CarmaJS.WidgetFramework.RouteFollowing = (function () {
        //Private variables
        var rect_index = 0;
        var totallanes = 0;
        var targetlane = 0;
        var currentlaneid = 0;

        var total_dist_next_lane_change = 0;

        //Listeners
        var listenerRouteState;
        var listenerRoute;

        //Currently the URL path from document or window are pointing to the page, not the actual folder location.
        //Therefore this needs to be hardcoded.
        //TODO: However, this could be set by widgetfw based on final install folder naming convention. Using _setOptions.
        var installfoldername = 'widgets/route_following/';

        //Private Functions
        /*
            Listen to route state to determine current, total, target lanes.
            Assumes that ROSLIB object has been initalized.
        */
        var checkRouteState = function () {

            //Get Route State
            listenerRouteState = new ROSLIB.Topic({
                ros: ros,
                name: t_route_state,
                messageType: 'cav_msgs/RouteState'
            });

            listenerRouteState.subscribe(function (message) {

                //if this div exists
                var divLaneDistRemaining = document.getElementById('divLaneDistRemaining');

                if (divLaneDistRemaining != 'undefined' && divLaneDistRemaining!= null)
                {
                    //Calculate and show next lane change remaining distance
                    //Show 0 if negative
                    var lane_remaining_dist = total_dist_next_lane_change - message.down_track;
                    lane_remaining_dist = Math.max(0, lane_remaining_dist);

                    var lane_remaining_dist_miles = (lane_remaining_dist * meter_to_mile);
                    lane_remaining_dist_miles = Math.max(0, lane_remaining_dist_miles);

                    insertNewTableRow('tblSecondA', 'Lane Change Total Dist (m)', total_dist_next_lane_change.toFixed(2));
                    insertNewTableRow('tblSecondA', 'Lane Change In (mi/m)', lane_remaining_dist_miles.toFixed(2) + ' mi / ' + lane_remaining_dist.toFixed(0) + ' m');

                    divLaneDistRemaining.innerHTML = 'Lane Change In: ' + lane_remaining_dist_miles.toFixed(2) + ' mi / ' + lane_remaining_dist.toFixed(0) + ' m';
                }

                //Get current lane
                if (message.lane_index != null && message.lane_index != 'undefined') {
                    currentlaneid = parseInt(message.lane_index);
                }

                //Get totallanes and targetlane
                if (message.current_segment.waypoint.lane_count != null
                    && message.current_segment.waypoint.lane_count != 'undefined') {

                    totallanes = parseInt(message.current_segment.waypoint.lane_count);
                    targetlane = parseInt(message.current_segment.waypoint.required_lane_index);

                    //When targetlane = -1, it is unavailable.
                    if (targetlane >= 0)
                        drawLanes(false, true);
                }

                //Determine the remaining distance to current lane
                if (sessionStorage.getItem('routeLaneChangeDist') != null) {
                    var routeLaneChangeDist = sessionStorage.getItem('routeLaneChangeDist');
                    routeLaneChangeDist = JSON.parse(routeLaneChangeDist);

                    //Loop thru to find the correct totaldistance
                    for (i = 0; i < routeLaneChangeDist.length; i++) {
                        if (message.current_segment.waypoint.waypoint_id <= routeLaneChangeDist[i].waypoint_id) {
                            total_dist_next_lane_change = routeLaneChangeDist[i].total_length;
                            break;
                        }
                    }
                    //insertNewTableRow('tblSecondA', 'total_dist_next_lane_change', total_dist_next_lane_change);
                }

                //Display the lateset route name and timer.
                var divRouteInfo = document.getElementById('divRouteInfo');
                if (divRouteInfo != null || divRouteInfo != 'undefined')
                    divRouteInfo.innerHTML = route_name + ' : ' + engaged_timer;
            });
        };

       /*
             Listen to route state to calculate the distance.
        */
        var showActiveRoute = function () {

            //Get Route State
            listenerRoute = new ROSLIB.Topic({
                ros: ros,
                name: t_active_route,
                messageType: 'cav_msgs/Route'
            });

            listenerRoute.subscribe(function (message) {

                //If nothing on the list, set all selected checkboxes back to blue (or active).
                if (message.segments == null || message.segments.length == 0) {
                    return;
                }

                //alert('showActive Route: sessionStorage.getItem(routeLaneChangeDist: ' + sessionStorage.getItem('routeLaneChangeDist'));
                if (sessionStorage.getItem('routeLaneChangeDist') == null) {
                    message.segments.forEach(calculateDistToNextLaneChange);
                }

            });
        };

        /*
            Calculate the distance to next lane change.
        */
        var calculateDistToNextLaneChange = function (segment) {

            if (segment == null)
                return;

            if (segment.length == null || segment.length == 'undefined' || segment.length <= 0)
                return;

            //To calculate the distance to next next lane change.
            var routeLaneChange; //To store the total distance for each lane change.
            var routeLaneChangeDist;

            if (sessionStorage.getItem('routeLaneChangeDist') == null) {
                routeLaneChange = { waypoint_id: segment.prev_waypoint.waypoint_id, total_length: segment.length, required_lane_index: segment.prev_waypoint.required_lane_index };
                routeLaneChangeDist = [];
                routeLaneChangeDist.push(routeLaneChange);

                sessionStorage.setItem('routeLaneChangeDist', JSON.stringify(routeLaneChangeDist));
            }
            else {

                routeLaneChangeDist = sessionStorage.getItem('routeLaneChangeDist');
                routeLaneChangeDist = JSON.parse(routeLaneChangeDist);

                var lastItem = routeLaneChangeDist[routeLaneChangeDist.length - 1];

                //If lane changes, add to list
                if (lastItem.required_lane_index != segment.waypoint.required_lane_index) {
                    routeLaneChange = {
                        waypoint_id: segment.waypoint.waypoint_id
                        , total_length: (lastItem.total_length + segment.length) //make this a running total for every speed limit change
                        , required_lane_index: segment.waypoint.required_lane_index
                    };
                    routeLaneChangeDist.push(routeLaneChange);

                    sessionStorage.setItem('routeLaneChangeDist', JSON.stringify(routeLaneChangeDist));
                }
                else //Update last item's length to the total length
                {
                    lastItem.waypoint_id = segment.waypoint.waypoint_id;
                    lastItem.total_length += segment.length;

                    sessionStorage.setItem('routeLaneChangeDist', JSON.stringify(routeLaneChangeDist));
                }
            }
        };

       /*
                Draw the lanes after route selection.
                addNew = true means to add a new rectangle. false means to clear the drawing area.
                clear = true means to clear the drawing area.
       */
        var drawLanes = function (addNew, clear) {

                var draw;

                if (addNew == true || SVG.get('svgEditorBackground') == null) {
                    //Check if SVG is supported
                    draw = SVG('divLaneTracking').size(600, 190).viewbox(0, 0, 600, 190).attr({
                        preserveAspectRatio: 'xMidYMid meet'
                        , id: 'svgEditorBackground'
                    }); //static to fit available space.
                }
                else {
                    draw = SVG.get('svgEditorBackground');
                }

                if (draw == null) {
                    document.getElementById('divLog').innerHTML += '<br/> Draw is null.';
                    return;
                }

                if (clear == true || addNew == false) {
                    draw.clear();
                }

                totallanes = parseInt(totallanes);
                targetlane = parseInt(targetlane);
                currentlaneid = parseInt(currentlaneid);


                var y1 = 0; //static height;
                var y2 = 189;//static height;
                var lanewidth = 109; //static lane width
                var firstline_x = 0; //default

                //e1_line.x1 = Starting point (currently hardcoded, cannot find the proper calc or pattern)
                //alert(totallanes);
                switch (totallanes) {
                    case 0:
                        break; //ignore as this is usually during initialization.
                    case 1:
                        firstline_x = 235;
                        break;
                    case 2:
                        firstline_x = 188;
                        break;
                    case 3:
                        firstline_x = 128;
                        break;
                    case 4:
                        firstline_x = 81;
                        break;
                    case 5:
                        firstline_x = 34;
                        break;
                    default:
                        console.log('UI cannot handle more than 5 lanes at this time.')
                        return; // cannot handle more than 5 lanes.
                }

                //x = e1_line.x1 - 10px (which is 10 px on each side); 81-71 = 10px
                //width = ((109 *#oflane) + 20px)
                var rect = draw.rect(456, 189).attr({
                    id: 'r' + rect_index //NOTE: if removed, it adds new rect after the first.
                    , x: firstline_x - 10 //225 //adjusted based on # of lanes
                    , y: 0
                    , width: (lanewidth * totallanes) + 20 //129 //adjusted based on # of lanes
                    , height: 189
                    , style: 'fill:gray; stroke: none;'
                });

                var currentline = firstline_x;


                for (i = totallanes; i >= 0; i--) {

                    var line = draw.line(currentline, y1, currentline, y2).attr({
                        id: 'r' + rect_index + 'e' + i + '_line'
                        //, style: 'stroke: white; fill: none; stroke-width: 5px;'
                    });

                    //alert(line.attr('id'));

                    //TODO: update based on topic
                    if (i == 0 || i == totallanes) {
                        // solid lines on the ends
                        line.attr({ style: 'stroke: white; fill: none; stroke-width: 5px;' });
                    } else {
                        // middle lines to be dashes
                        line.attr({ style: 'stroke: white; fill: none; stroke-width: 5px; stroke-dasharray:15px, 15px;' });
                    }

                    currentline = currentline + lanewidth;
                }

                /*** Color Target Lane
                    <rect x="190" y="0" width="109" height="217" style="fill:lime;fill-opacity:0.22;" id="rect_current_lane" />
                */
                if (targetlane >= 0) {
                    var lineid = 'r' + rect_index + 'e' + (targetlane + 1) + '_line';
                    var targetline = SVG.get(lineid);

                    if (targetline != null) {
                        var line_x = targetline.attr('x1');
                        var targetlanestyle;

                        if (currentlaneid == targetlane)
                            targetlanestyle = 'fill:#4CAF50; fill-opacity:1.00;'; //Green
                        else
                            targetlanestyle = 'fill:cornflowerblue; fill-opacity:1.00;'; //Blue

                        //static width 109 and heigh 217
                        var rect_target = draw.rect(109, 217).attr({
                            id: 'rect_target_lane'
                            , x: line_x
                            , y: 0
                            , style: targetlanestyle
                        });
                    }
                }

                //Current Lane
                //currentlaneid is 0 based.
                if (currentlaneid >= 0) {

                    var lineid = 'r' + rect_index + 'e' + (currentlaneid + 1) + '_line';
                    var targetline = SVG.get(lineid);

                    if (targetline != null) {
                        var line_x = targetline.attr('x1');

                        //To calculate image x position = e#_line.x1 - 19px
                        //TODO: Convert to svg to change colors.
                        var image = draw.image(installfoldername + 'SUV.png', 145.136, 157.036);
                        // another option is to reuse what's already in carma/images.and just have 'images/SUV.png'

                        image.attr({
                            id: 'r' + rect_index + '_image'
                            , x: line_x - 19
                            , y: 19.6868
                            , visibility: 'visible'
                        });
                    }
                }

                //Display OFF ROAD when less than 0 or >= totallanes
                if (currentlaneid < 0 || currentlaneid >= totallanes) {
                    var text = draw.text(function (add) {
                        add.tspan('OFF ROAD').fill('#b32400').style('font-weight:bold;font-size:20px;') /* red font */
                    });

                    //center accordingly to # of lanes.
                    if (totallanes == 3 || totallanes == 1)
                        text.center(290, 95);
                    else
                        text.center(305, 95);
                }

                //increment rect at the end
                rect_index++;
            };

        /***
        * Custom widgets using JQuery Widget Framework.
        * NOTE: that widget framework namespace can only be one level deep.
        ***/
        $.widget("CarmaJS.rfLaneTracking", {
           _create: function() {
              var myDiv = $("<div id='divLaneTracking'></div><br/>"
                          + "<div id='divLaneDistRemaining'></div>");

              //this._div = $("<button>");
              $(this.element).append(myDiv);
           },
           _destroy: function() {
                if (listenerRouteState)
                    listenerRouteState.unsubscribe();
                if (listenerRoute)
                   listenerRoute.unsubscribe();
                this.element.empty();
                this._super();
           },
           draw: function() {
              drawLanes(false, true);
           },
           checkRouteState: function(){
              checkRouteState();
           },
           showActiveRoute: function(){
              showActiveRoute();
           }
        });

        $.widget("CarmaJS.rfRouteInfo", {
           _create: function() {
              var myDiv = $(" <div id='divRouteInfo'>No Route Selected : 00h 00m 00s</div>");
              $(this.element).append(myDiv);
           },
           _destroy: function () {
                this.element.empty();
                this._super();
           }
        });

        /*** Public Functions ***
        This is the main call to setup the different widgets available in this plugin.
        ***/
        var loadCustomWidget = function(container) {

            //Generate the  widget and calling its private method(s) below.
            container.rfRouteInfo();

            container.rfLaneTracking();
            container.rfLaneTracking("showActiveRoute", null);
            container.rfLaneTracking("checkRouteState", null);
            container.rfLaneTracking("draw", null);
        };

        //Public API
        return {
            loadCustomWidget: loadCustomWidget
        };
})();