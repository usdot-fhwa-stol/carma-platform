/***
* Create a unique namespace for each plugin widget to minimize collision of same name variables or functions.
***/
CarmaJS.registerNamespace("CarmaJS.WidgetFramework.CACCPlatooning");

CarmaJS.WidgetFramework.CACCPlatooning = (function () {
        //Private variables
        var t_platooning_info = '/saxton_cav/guidance/platooning_info';
        var hostDowntrackDistance = 0;

        //Listeners
        var listenerRouteState;
        var listenerPlatooning;

        //Currently the URL path from document or window are pointing to the page, not the actual folder location.
        //Therefore this needs to be hardcoded.
        //TODO: However, this could be set by widgetfw based on final install folder naming convention. Using _setOptions.
        var installfoldername = 'widgets/cacc_platooning/';

        //Private Functions
        /*
            Listen to route state to determine current, total, target lanes.
            Assumes that ROSLIB object has been initalized.
        */
        var checkRouteState = function () {

            listenerRouteState = new ROSLIB.Topic({
                ros: ros,
                name: t_route_state,
                messageType: 'cav_msgs/RouteState'
            });

            listenerRouteState.subscribe(function (message) {

                //store the down_track, and calculate the gap distance on the platooning info.
                hostDowntrackDistance = message.down_track;
            });
        };

       /*
             Listen to platooning state and calculate the distance.
        */
        var checkPlatooningInfo = function () {

            // List out the expected state to handle.
            var platooningInfoState = {
                STANDBY: { value: 0, text: 'STANDBY' },
                LEADER: { value: 1, text: 'LEADER' },
                FOLLOWER: { value: 2, text: 'FOLLOWER' },
                //Add new ones here.
            };

            listenerPlatooningInfo = new ROSLIB.Topic({
                ros: ros,
                name: t_platooning_info,
                messageType: 'cav_msgs/PlatooningInfo'
            });

            listenerPlatooningInfo.subscribe(function (message) {

                var leader_cmd_speed_mph = Math.round(message.leader_cmd_speed * METER_TO_MPH);
                var host_cmd_speed_mph = Math.round(message.host_cmd_speed * METER_TO_MPH);
                var host_actual_gap = message.leader_downtrack_distance - hostDowntrackDistance;
                var platoonStateText;

                switch (message.state) {
                    case platooningInfoState.STANDBY.value:
                        platoonStateText = platooningInfoState.STANDBY.text;
                        break;
                    case platooningInfoState.LEADER.value:
                        platoonStateText = platooningInfoState.LEADER.text;
                        break;
                    case platooningInfoState.FOLLOWER.value:
                        platoonStateText = platooningInfoState.FOLLOWER.text;
                        break;
                    default:
                        platoonStateText = 'xxxxxx';
                        break;
                }

                insertNewTableRow('tblFirstB', 'Platoon ID', message.platoon_id);
                insertNewTableRow('tblFirstB', 'Platoon State', message.state);
                insertNewTableRow('tblFirstB', 'Platoon Size', message.size);
                insertNewTableRow('tblFirstB', 'Platoon Size Limit', message.size_limit);
                insertNewTableRow('tblFirstB', 'Platoon Leader ID', message.leader_id);
                insertNewTableRow('tblFirstB', 'Platoon Host Position', message.host_platoon_position);
                insertNewTableRow('tblFirstB', 'Platoon Leader Cmd Speed', leader_cmd_speed_mph);
                insertNewTableRow('tblFirstB', 'Platoon Host Cmd Speed', host_cmd_speed_mph);
                insertNewTableRow('tblFirstB', 'Platoon Leader Downtrack', message.leader_downtrack_distance.toFixed(2));
                insertNewTableRow('tblFirstB', 'Platoon Host Downtrack', hostDowntrackDistance.toFixed(2));
                insertNewTableRow('tblFirstB', 'Platoon Desired Gap', message.desired_gap.toFixed(2));
                insertNewTableRow('tblFirstB', 'Platoon Actual Gap', host_actual_gap.toFixed(2));

                if (document.getElementById('divPlatoonId') == null)
                    return;

                document.getElementById('divPlatoonId').innerHTML = 'Platoon ID: ' + message.platoon_id;
                document.getElementById('divPlatoonState').innerHTML = 'State: ' + platoonStateText;

                document.getElementById('divHostTitle').innerHTML =  'Host <br/> ' + message.host_platoon_position + ' out of ' + message.size;
                document.getElementById('divLeaderTitle').innerHTML =  'Lead <br/>' + message.leader_id ;

                document.getElementById('divHostCmdSpeed').innerHTML = host_cmd_speed_mph;
                document.getElementById('divLeadCmdSpeed').innerHTML = leader_cmd_speed_mph;

                document.getElementById('txtDesiredGap').innerHTML = 'Desired: ' + message.desired_gap.toFixed(2) +  ' meters';
                document.getElementById('txtActualGap').innerHTML = 'Actual: ' + host_actual_gap.toFixed(2) + ' meters';

                $('#gaugeDesiredGap').jqxLinearGauge('value', message.desired_gap.toFixed(2));
                $('#gaugeActualGap').jqxLinearGauge('value', host_actual_gap.toFixed(2));

            });
        };


        /***
        * Custom widgets using JQuery Widget Framework.
        * NOTE: that widget framework namespace can only be one level deep.
        ***/
        $.widget("CarmaJS.platooningDistanceGap", {
           _create: function() {

                  //Add styles and scripts dynamically. These references are already part of the website.
                  var appendToHeader = $("<script type='text/javascript' src='thirdparty/jqwidgets/jqxcore.js'></script>"
                                       + "<script type='text/javascript' src='thirdparty/jqwidgets/jqxdata.js'></script>"
                                       + "<script type='text/javascript' src='thirdparty/jqwidgets/jqxexpander.js'></script>"
                                       + "<script type='text/javascript' src='thirdparty/jqwidgets/jqxdraw.js'></script>"
                                       + "<script type='text/javascript' src='thirdparty/jqwidgets/jqxgauge.js'></script>"
                                      );
                 $('head').append(appendToHeader);

               var myDiv = $("<div id='divPlatoonId' class='divPlatoonStyle1'>Platoon ID: xxxxxxxx-xxxx-xxxx-xxxx-xxxxxxxxxxxx </div>"
                           + "<div id='divPlatoonState' class='divPlatoonStyle1'>State: xxxxxx </div>"
                           + "<div class='leftRelative'>"
                           + "    <img id='imgHostVehicle' src='images/SUV_64x38.png'/>"
                           + "    <div id='divHostTitle'>Host <br/> # out of #</div>"
                           + "    <div id='divHostCmdSpeed' class='numberCircle'>##</div>"
                           + "</div>"
                           + "<div class='leftRelative'>"
                           + "    <div id='gaugeActualGap'></div>"
                           + "    <div id='gaugeDesiredGap'></div>"
                           + "    <div id='txtActualGap'>Actual: #.## meters</div>"
                           + "    <div id='txtDesiredGap'>Desired: #.## meters</div>"
                           + "</div>"
                           + "<div class='leftRelative divWidth' >"
                           + "    <img id='imgLeadVehicle' src='images/SUV_64x38.png' />"
                           + "    <div id='divLeaderTitle'>Lead<br/> DOT-##### </div>"
                           + "    <div id='divLeadCmdSpeed' class='numberCircle'>##</div>"
                           + "</div>"
                           );

               $(this.element).append(myDiv);
           },
           _destroy: function() {
                if (listenerRouteState)
                    listenerRouteState.unsubscribe();
                if (listenerPlatooning)
                   listenerPlatooning.unsubscribe();
                this.element.empty();
                this._super();
           },
           drawGuages: function(){

              //*** Draw the guages and labels ***
              //console.log('drawGuages!!!');
              //Top linear guage is the actual gap with ticker labels.
              var majorTicks = { size: '17%', interval: 10 },
                  minorTicks = { size: '12%', interval: 5, style: { 'stroke-width': 1, stroke: '#aaaaaa' } },
                  labels = { interval: 10, position: 'near' };

              $('#gaugeActualGap').jqxLinearGauge({
                  orientation: 'horizontal',
                  height: '100px',
                  width: '300px',
                  labels: labels,
                  ticksMajor: majorTicks,
                  ticksMinor: minorTicks,
                  ticksPosition: 'near',
                  min: 0,
                  max: 100,
                  value: 0,
                  pointer: { size: '6%' },
                  colorScheme: 'scheme06', //black
                  background: { visible: false },
                  showRanges: true,
                  ranges: [
                      { startValue: 0, endValue: 20, style: { fill: '#FF4800', stroke: '#FF4800' } },
                      { startValue: 20, endValue: 80, style: { fill: '#FFA200', stroke: '#FFA200' } },
                      { startValue: 80, endValue: 100, style: { fill: '#FFF157', stroke: '#FFF157' } }]
              });

              //Bottom linear guage is the desired gap with tickers but with no labels.
              var majorTicks2 = { size: '15%', interval: 10 },
                  minorTicks2 = { size: '10%', interval: 2, style: { 'stroke-width': 1, stroke: '#aaaaaa' } },
                  labels2 = { interval: 5, position: 'near' };

              $('#gaugeDesiredGap').jqxLinearGauge({
                  orientation: 'horizontal',
                  height: '100px',
                  width: '300px',
                  labels: { visible: false },
                  ticksMajor: majorTicks2,
                  ticksMinor: minorTicks2,
                  ticksPosition: 'far',
                  min: 0,
                  max: 100,
                  value: 0,
                  pointer: { size: '6%' },
                  colorScheme: 'scheme02', //green
                  background: { visible: false },
                  showRanges: false,
                  ranges: [
                      { startValue: 0, endValue: 10, style: { fill: '#FFF157', stroke: '#FFF157' } },
                      { startValue: 10, endValue: 40, style: { fill: '#FFA200', stroke: '#FFA200' } },
                      { startValue: 40, endValue: 50, style: { fill: '#FF4800', stroke: '#FF4800' } }]
              });
           },
           checkRouteState: function(){
              checkRouteState();
           },
           checkPlatooningInfo: function(){
              checkPlatooningInfo();
           }
        });


        /*** Public Functions ***
        This is the main call to setup the different widgets available in this plugin.
        **/
        var loadCustomWidget = function(container) {

                //Generate the  widget and calling its private method(s) below.
                container.platooningDistanceGap();
                container.platooningDistanceGap("checkRouteState", null);
                container.platooningDistanceGap("checkPlatooningInfo", null);
                container.platooningDistanceGap("drawGuages", null);

        };

        //Public API
        return {
            loadCustomWidget: loadCustomWidget
        };
})();