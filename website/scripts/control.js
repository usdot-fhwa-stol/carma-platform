/*
 * Copyright (C) 2017 LEIDOS.
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
 This file shall contain generic manipulation of html elements.
 Do not to mix ROS functionality here.
****/

/*
* Opens the selected Tab.
*/
function openTab(evt, name) {
    var i, tabcontent, tablinks;
    tabcontent = document.getElementsByClassName('tabcontent');
    for (i = 0; i < tabcontent.length; i++) {
        tabcontent[i].style.display = 'none';
    }
    tablinks = document.getElementsByClassName('tablinks');
    for (i = 0; i < tablinks.length; i++) {
        tablinks[i].className = tablinks[i].className.replace(' active', '');
    }
    document.getElementById(name).style.display = 'block';
    evt.currentTarget.className += ' active';

    //Initialize the map.
    if (name == 'divMap')
        initMap();
}

// Get the element with id="defaultOpen" and click on it
// This needs to be outside a funtion to work.
document.getElementById('defaultOpen').click();

/*
* Adds a new radio button onto the container.
*/
function createRadioElement(container, radioId, radioTitle, itemCount, groupName) {

    var newInput = document.createElement('input');
    newInput.type = 'radio';
    newInput.name = groupName;

    //TODO: Remove this when RouteID has been changed to have no spaces.
    //Currently, RouteID and RouteName are same and have spaces, but ID should not have any spaces. For now, updating to have underscore
    var revisedId = radioId.toString().replace( / /g, '_' );
    newInput.id = 'rb' + revisedId;
    newInput.onclick = function () { setRoute(newInput.id.toString()) };

    var newLabel = document.createElement('label');
    newLabel.id = 'lbl' + revisedId;
    newLabel.htmlFor = newInput.id.toString();
    newLabel.innerHTML = radioTitle;

    // Add the new elements to the container
    container.appendChild(newInput);
    container.appendChild(newLabel);
}

/*
* Adds a new checkbox onto the container.
*/
function createCheckboxElement(container, checkboxId, checkboxTitle, itemCount, groupName, isChecked, isRequired) {

    var newInput = document.createElement('input');
    newInput.type = 'checkbox';
    newInput.name = groupName;
    newInput.id = 'cb' + checkboxId.toString();
    newInput.checked = isChecked;
    newInput.onclick = function () { activatePlugin(newInput.id.toString()) };

    var newLabel = document.createElement('label');
    newLabel.id = 'lbl' + checkboxId.toString();
    newLabel.htmlFor = newInput.id.toString();

    if (isRequired == true)
        newLabel.innerHTML = '<span style="color:#FF0000">* </span>'

    newLabel.innerHTML += checkboxTitle;


    container.appendChild(newInput);
    container.appendChild(newLabel);

    //var newDiv = document.createElement('div');
    //newDiv.id = 'div' + checkboxId.toString();

    // Add the new elements to the container
    //container.appendChild(newDiv);
    //newDiv.appendChild(newInput);
    //newDiv.appendChild(newLabel);

}

/*
* Get list of plugins selected by user and return count.
*/
function getCheckboxesSelected() {
    var cbResults = 'Selected Items: ';
    var count = 0;
    var allInputs = document.getElementsByTagName('input');
    for (var i = 0, max = allInputs.length; i < max; i++) {
        if (allInputs[i].type === 'checkbox') {
            if (allInputs[i].checked == true) {
                cbResults += allInputs[i].id + '; ';
                count++;
            }
        }
    }

    return count;
}

/*
* Sets the background color of all selected checkboxes.
*/
function setCbSelectedBgColor(color) {
    var allInputs = document.getElementsByTagName('input');

    for (var i = 0, max = allInputs.length; i < max; i++) {

        if (allInputs[i].type === 'checkbox') {
            if (allInputs[i].checked == true) {
                //find the label for the checkbox and update it's color
                var labelForCb = document.getElementById(allInputs[i].id.replace('cb', 'lbl'));
                labelForCb.style.backgroundColor = color;
            }//if
        }//if
    }//for
}

/*
* Sets the background color of a specific checkbox.
*/
function setCbBgColor(id, color) {
    //find the label for the checkbox and update it's color, this is only way to update the checkbox color.
    var lblName = 'lbl' + id;
    var labelForCb = document.getElementById(lblName);

    if (labelForCb != null)
        labelForCb.style.backgroundColor = color;

}

/*
* Add new row to the table.
*/
function insertNewTableRow(tableName, rowTitle, rowValue) {

    var cellId = rowTitle.replace(/ /g, '');

    var existingCell2 = document.getElementById(cellId);

    if (existingCell2 != null) {
        existingCell2.innerHTML = rowValue;
        existingCell2.className = 'col-style2b';
    }
    else {
        var table = document.getElementById(tableName);
        var row = table.insertRow(table.rows.count);
        var cell1 = row.insertCell(0);
        var cell2 = row.insertCell(1);

        cell1.innerHTML = rowTitle;
        cell2.innerHTML = rowValue;
        cell1.className = 'col-style2a hyphenate';
        cell2.className = 'col-style2b hyphenate';
        cell2.id = cellId;
    }
}

/*
    Set the values of the speedometer
*/
function setSpeedometer(speed)
{
    var maxMPH = 160;
    var deg = (speed/maxMPH)*180;
    document.getElementById('percent').innerHTML = speed;
    var element = document.getElementsByClassName('gauge-c')[0];

    element.style.webkitTransform = 'rotate(' + deg + 'deg)';
    element.style.mozTransform = 'rotate(' + deg + 'deg)';
    element.style.msTransform = 'rotate(' + deg + 'deg)';
    element.style.oTransform = 'rotate(' + deg + 'deg)';
    element.style.transform = 'rotate(' + deg + 'deg)';

}

/*
 Open the modal popup.
 TODO: Update to allow caution and warning message scenarios. Currently only handles fatal and guidance dis-engage which redirects to logout page.
*/
function showModal(showWarning, modalMessage) {

    //IF modal is already open, do not show another alert.
    if (isModalPopupShowing == true)
        return;

    var modal = document.getElementById('myModal');
    var span_modal = document.getElementsByClassName('close')[0];

    // When the user clicks on <span> (x), close the modal
    span_modal.onclick = function () {
        closeModal();
        return;
    }

    modal.style.display = 'block';

    var modalBody = document.getElementsByClassName('modal-body')[0];
    var modalHeader = document.getElementsByClassName('modal-header')[0];
    var modalFooter = document.getElementsByClassName('modal-footer')[0];

    if (showWarning == true)
    {
        modalHeader.innerHTML = '<span class="close">&times;</span><h2>SYSTEM ALERT<i class="fa fa-exclamation-triangle" style="font-size:40px; color:red;"></i></h2>';
        modalHeader.style.backgroundColor = '#ffcc00'; // yellow
        //modalHeader.style.color = 'black';
        modalFooter.style.backgroundColor = '#ffcc00';
        playSound('audioAlert1', true);
    }
    else
    {
        modalHeader.innerHTML = '<span class="close">&times;</span><h2>SUCCESS <i class="fa fa-smile-o" style="font-size:50px; font-bold: true;"></i></h2>';
        modalHeader.style.backgroundColor = '#4CAF50'; // green
        //modalHeader.style.color = 'white';
        modalFooter.style.backgroundColor = '#4CAF50';
        playSound('audioAlert2', true);
    }

    modalBody.innerHTML = '<p>' + modalMessage + '</p>';

    isModalPopupShowing = true; //flag that modal popup for an alert is currently being shown to the user.
}

/*
    Play sound for specific alerts or notifications.
*/
function playSound(audioId, repeat)
{
    var audioAlert1 = document.getElementById(audioId);
    audioAlert1.currentTime = 0;
    audioAlert1.play();

    if (repeat == false)
        return;

    //Repeat the sounds 5x max or until OK/logout page shows.
    setTimeout(function () {
        sound_counter++;
        if (sound_counter < sound_counter_max){
            playSound(audioId, true);
        }
     }, 3000)

}

/*
    Count up Timer for when Guidance is started.
*/

function countUpTimer() {

    // Get todays date and time
    var now = new Date().getTime();

    // Find the distance between now an the count down date
    var distance = now - startDateTime;

    // Time calculations for days, hours, minutes and seconds
    // var days = Math.floor(distance / (1000 * 60 * 60 * 24));
    var hours = Math.floor((distance % (1000 * 60 * 60 * 24)) / (1000 * 60 * 60));
    var minutes = Math.floor((distance % (1000 * 60 * 60)) / (1000 * 60));
    var seconds = Math.floor((distance % (1000 * 60)) / 1000);

    //Display the route name
    var divRouteInfo = document.getElementById('divRouteInfo');

    if (divRouteInfo != null)
    {
        divRouteInfo.innerHTML = route_name + ': ' + pad(hours,2) + "h "
                                 + pad(minutes,2) + "m " + pad(seconds,2) + "s ";
    }
}

function pad(num, size) {
	var s = "0000" + num;
	return s.substr(s.length - size);
}

/*
    Close the modal popup.
*/
function closeModal() {
    var modal = document.getElementById('myModal');
    modal.style.display = 'none';

    isModalPopupShowing = false; //flag that modal popup has been closed.

    document.getElementById('audioAlert1').pause();
    document.getElementById('audioAlert2').pause();
    document.getElementById('audioAlert3').pause();

    window.location.assign('logout.html');

}
