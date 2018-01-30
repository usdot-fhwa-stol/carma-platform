/*
 * Copyright (C) 2018 LEIDOS.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */

/***
    This file shall contain SVG relate function calls.
****/

/*
    Draw the lanes after route selection.
    addNew = true means to add a new rectangle. false means to clear the drawing area.
    clear = true means to clear the drawing area. 
*/
function drawLanes(addNew, clear) {

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
        //console.log('TARGET: lineid: ' + lineid +  '; x1: ' + targetline.attr('x1'));

        if (targetline != null) {
            var line_x = targetline.attr('x1');
            //console.log('TARGET: targetline.x1:' + line_x);

            var targetlanestyle;
            if (currentlaneid == targetlane)
                targetlanestyle = 'fill:#4CAF50; fill-opacity:0.22;'; //Green
            else
                targetlanestyle = 'fill:cornflowerblue; fill-opacity:0.22;';

            //static width 109 and heigh 217
            var rect_target = draw.rect(109, 217).attr({
                id: 'rect_target_lane'
                , x: line_x
                , y: 0
                , style: targetlanestyle
            });
        }
        else {
            console.log('TARGET: targetline is null!');
        }
    }

    /*** Current Lane
    */
    if (currentlaneid >= 0) {

        var lineid = 'r' + rect_index + 'e' + (currentlaneid + 1) + '_line';
        var targetline = SVG.get(lineid);
        //console.log('CURRENT: lineid: ' + lineid + '; x1: ' + targetline.attr('x1'));

        if (targetline != null) {
            var line_x = targetline.attr('x1');
            //console.log('CURRENT: targetline.x1:' + line_x);

            //To calculate image x position = e#_line.x1 - 19px
            //TODO: Convert to svg to change colors.
            var image = draw.image('../images/SUV.png', 145.136, 157.036);
            image.attr({
                id: 'r' + rect_index + '_image'
                , x: line_x - 19
                , y: 19.6868
                , visibility: 'visible'
            });
        }
        else {
            console.log('CURRENT: targetline is null!');
        }
    }

    //Display OFF ROAD when less than 0 or >= totallanes
    //currentlaneid is 0 based.
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
}
