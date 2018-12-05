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

        //If DriverView, collapse left navigation bar.
        if (name != 'divCapabilities')
            tabcontent[i].style.width = '88%';
        else
            tabcontent[i].style.width = '82%';

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

    //If DriverView, collapse left navigation bar.
    var tabname = document.getElementsByClassName('tabName');

    for (i = 0; i < tabname.length; i++) {

        //If DriverView, collapse left navigation bar.
        if (name != 'divCapabilities')
            tabname[i].style.display = 'none';
        else
            tabname[i].style.display = '';
    }

    var tab = document.getElementsByClassName('tab');
    var imgCARMAIconColor = document.getElementById('imgCARMAIconColor');

    for (i = 0; i < tab.length; i++) {

        //If not Task View, collapse left navigation bar.
        if (name != 'divCapabilities')
        {
            tab[i].style.width = '6%';
            imgCARMAIconColor.style.width='55px';
            imgCARMAIconColor.style.height= 'auto';
            imgCARMAIconColor.style.padding = '20px 0px 0px 7px';
        }
        else
        {
            tab[i].style.width = '12%';
            imgCARMAIconColor.style.width='120px';
            imgCARMAIconColor.style.height= '120px';
            imgCARMAIconColor.style.padding = '20px 0px 0px 15px';
        }
    }

    var tabheader = document.getElementsByClassName('tabheader');

    for (i = 0; i < tabheader.length; i++) {

        //If not Task View, adjust the button size.
        if (name != 'divCapabilities')
            tabheader[i].style.width = '88%';
        else
            tabheader[i].style.width = '82%';
    }


}

// Get the element with id="defaultOpen" and click on it
// This needs to be outside a funtion to work.
document.getElementById('defaultOpen').click();

/*
* Adds a new radio button onto the container.
*/
function createRadioElement(container, radioId, radioTitle, itemCount, groupName, isValid) {

    var newInput = document.createElement('input');
    newInput.type = 'radio';
    newInput.name = groupName;

    //TODO: Remove this when RouteID has been changed to have no spaces.
    //Currently, RouteID and RouteName are same and have spaces, but ID should not have any spaces. For now, updating to have underscore
    var revisedId = radioId.toString().replace(/ /g, '_');
    newInput.id = 'rb' + revisedId;
    newInput.onclick = function () { setRoute(newInput.id.toString()) };

    var newLabel = document.createElement('label');
    newLabel.id = 'lbl' + revisedId;
    newLabel.htmlFor = newInput.id.toString();
    newLabel.innerHTML = radioTitle;

    //If this field is false then the UI should mark the button and make it unselectable.
    if (isValid == false)
    {
         newInput.disabled = true;
         newLabel.innerHTML += '&nbsp; <i class="fa fa-ban" style="color:#b32400";></i>';
    }

    // Add the new elements to the container
    container.appendChild(newInput);
    container.appendChild(newLabel);
}

/*
* Adds a new checkbox onto the container.
*/
function createCheckboxElement(container, checkboxId, checkboxTitle, itemCount, groupName, isChecked, isRequired, funcName) {

    var alreadyExists = document.getElementById(checkboxId);

    if (alreadyExists != null)
        return;

    var newInput = document.createElement('input');
    newInput.type = 'checkbox';
    newInput.name = groupName;
    newInput.id = 'cb' + checkboxId;
    newInput.checked = isChecked;
    newInput.onclick = function () { eval(funcName) (newInput.id) }; //e.g. function () { activatePlugin(newInput.id) };
    newInput.setAttribute('title', checkboxTitle);

    var newLabel = document.createElement('label');
    newLabel.id = 'lbl' + checkboxId;
    newLabel.htmlFor = newInput.id;

    if (isRequired == true)
        newLabel.innerHTML = '<span style="color:#FF0000">* </span>'

    newLabel.innerHTML += checkboxTitle;


    container.appendChild(newInput);
    container.appendChild(newLabel);

}

/*
* Get list of plugins selected by user and return count.
*/
function getCheckboxesSelected(container) {

    if (container == null || container == 'undefined')
        return;

    //var cbResults = 'Selected Items: ';
    var count = 0;
    var pluginList = [];

    var allInputs = container.getElementsByTagName('input');

    for (var i = 0, max = allInputs.length; i < max; i++) {
        if (allInputs[i].type === 'checkbox') {
            if (allInputs[i].checked == true) {
                //cbResults += allInputs[i].id + '; ';
                pluginList.push (allInputs[i]);
                count++;
            }
        }
    }

    return pluginList;
}

/*
    Find the checkbox by Id within the DIV
*/
function checkboxExistsById(container, id) {
    if (container == null || container == 'undefined')
        return;

    //var cbResults = 'Selected Items: ';
    var count = 0;
    var pluginList = [];

    var allInputs = container.getElementsByTagName('input');

    for (var i = 0, max = allInputs.length; i < max; i++) {
        if (allInputs[i].type === 'checkbox') {
            if (allInputs[i].id == id)
            {
                return true;
            }
        }
    }
    return false;
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
* Remove all rows except first 2 row header
*/
function clearTable(tableName) {

    var myTable = document.getElementById(tableName);
    var noOfRows = myTable.rows.length - 1;
    //alert(noOfRows);

    for (i = noOfRows; i >= 2; i--) {
        //alert(i);
        myTable.deleteRow(i);
    }
}

/* Open the modal UIInstructions when there's no acknowledgement needed.
   Currently only handling lane change.
*/
function showModalNoAck(msg) {

    //IF modal is already open, skip;
    if (isModalPopupShowing == true)
        return;

    //show modal
    var modalUIInstructions = document.getElementById('modalUIInstructions');
    var modalUIInstructionsContent = document.getElementById('modalUIInstructionsContent');
    modalUIInstructionsContent.innerHTML = icon;
    modalUIInstructions.style.display = 'block';
    isModalPopupShowing = true;
    
    playSound('audioAlert4', false);

    //hide after 3 seconds.
   setTimeout(function(){
        modalUIInstructions.style.display = '';
        modalUIInstructionsContent.innerHTML = '';
        isModalPopupShowing = false;
   }, 3000);
}

/* Open the modal UIInstructions when there is an acknowledgement needed. */
function showModalAck(msg, response_service) {

    //IF modal is already open, do not show another alert.
    if (isModalPopupShowing == true)
        return;

    var modal = document.getElementById('modalMessageBox');
    var span_modal = document.getElementsByClassName('close')[0];
    var btnModalButton1 = document.getElementById('btnModalButton1');
    var btnModalButton2 = document.getElementById('btnModalButton2');
    var modalFooterMessage = document.getElementById('divFooterMessage');

    modalFooterMessage.innerHTML = ''; //Clear every time.

    btnModalButton1.title = 'YES';
    btnModalButton1.innerHTML = 'YES';

    btnModalButton1.onclick = function () {
        sendModalResponse(true, response_service);
        return;
    }

    btnModalButton2.title = 'NO';
    btnModalButton2.innerHTML = 'NO';
    btnModalButton2.onclick = function () {
        sendModalResponse(false, response_service);
        return;
    }

    // When the user clicks on <span> (x), close the modal
    span_modal.onclick = function () {
        sendModalResponse(false, response_service);
        return;
    }

    //display the modal
    modal.style.display = 'block';

    var modalBody = document.getElementsByClassName('modal-body')[0];
    var modalHeader = document.getElementsByClassName('modal-header')[0];
    var modalFooter = document.getElementsByClassName('modal-footer')[0];

    modalHeader.innerHTML = '<span class="close">&times;</span><h2>ACTION REQUIRED &nbsp; <i class="fa fa-exclamation-triangle" style="font-size:40px; color:red;"></i></h2>';
    modalHeader.style.backgroundColor = '#ffcc00'; // yellow
    modalFooter.style.backgroundColor = '#ffcc00';
    playSound('audioAlert1', true);

    modalBody.innerHTML = '<p>' + msg + '</p>';

    isModalPopupShowing = true; //flag that modal popup for an alert is currently being shown to the user.

}


/*
    Send the modal popup response back to the plugin.
*/
function sendModalResponse(operatorResponse, serviceName) {

    //send the service response
    //alert('sendModalResponse: ' + operatorResponse + 'serviceName: ' + serviceName);

    // Calling a service
    // -----------------
    // First, we create a Service client with details of the service's name and service type.
    var serviceClient = new ROSLIB.Service({
    ros : ros,
    name : serviceName,
    serviceType : 'std_srvs/SetBool'
    });

    // Then we create a Service Request. The object we pass in to ROSLIB.ServiceRequest matches the
    // fields defined in the .srv file.
    var serviceRequest = new ROSLIB.ServiceRequest({
    data : operatorResponse
    });

    // Finally, we call the service and get back the results in the callback. The result
    // is a ROSLIB.ServiceResponse object.
    serviceClient.callService(serviceRequest, function(result) {

        console.log('Result for service call on ' + serviceClient.name + ': ' + result.success + '; Boolean(result.success): ' + Boolean(result.success)  + '; message: ' + result.message);

        var modalFooterMessage = document.getElementById('divFooterMessage'); //Added specifically for returned messages.
        modalFooterMessage.innerHTML = ''; //Clear every time.

        //UI expects service to return true, if no errors occurred during processing.
        //If there was, then service should return false with a brief message explanation.
        if (Boolean(result.success) == true)
        {
            var modal = document.getElementById('modalMessageBox');
            modal.style.display = 'none';

            isModalPopupShowing = false; //flag that modal popup has been closed.

            //pause any sounds.
            document.getElementById('audioAlert1').pause();
            document.getElementById('audioAlert2').pause();
            document.getElementById('audioAlert3').pause();
            document.getElementById('audioAlert4').pause();

            if (result.message != '')
                modalFooterMessage.innerHTML = result.message;
        }
        else
        {
            if (result.message != '')
                modalFooterMessage.innerHTML = result.message;
        }
    }, function(error) {

          console.log("Calling service " + operatorResponse + " failed with error: " + error); // TODO we need to identify how to handle failure

          var modalFooterMessage = document.getElementById('divFooterMessage'); //Added specifically for returned messages.
          modalFooterMessage.innerHTML = ''; //Clear every time.

          modalFooterMessage.innerHTML = 'Error with service call:' + operatorResponse + ': ' + error;

    });

}

/*
 Open the modal popup.
 TODO: Update to allow caution and warning message scenarios. Currently only handles fatal and guidance dis-engage which redirects to logout page.
 showWarning = true : Shows the warning icon in red.
 showWarning = false: Succesful completion.
 restart = true: return to main screen.
*/
function showModal(showWarning, modalMessage, restart) {

    //IF modal is already open, do not show another alert.
    if (isModalPopupShowing == true)
        return;

    var modal = document.getElementById('modalMessageBox');
    var span_modal = document.getElementsByClassName('close')[0];
    var btnModalButton1 = document.getElementById('btnModalButton1');
    var btnModalButton2 = document.getElementById('btnModalButton2');
    var modalFooterMessage = document.getElementById('divFooterMessage');

    modalFooterMessage.innerHTML = ''; //Clear every time.

    btnModalButton1.title = 'Continue to Restart';
    btnModalButton2.title = 'Logout and Shutdown';

    btnModalButton1.innerHTML = 'Continue';
    btnModalButton2.innerHTML = 'Logout';

    btnModalButton1.onclick = function () {
        closeModal('RESTART');
        return;
    }

    btnModalButton2.onclick = function () {
        closeModal('LOGOUT');
        return;
    }

    if (restart == true) {
        btnModalButton1.style.display = '';
        btnModalButton2.style.display = '';

        // When the user clicks on <span> (x), close the modal
        span_modal.onclick = function () {
            closeModal('RESTART');
            return;
        }
    }
    else {
        btnModalButton1.style.display = 'none';
        btnModalButton2.style.display = '';

        // When the user clicks on <span> (x), close the modal
        span_modal.onclick = function () {
            closeModal('LOGOUT');
            return;
        }
    }

    //display the modal
    modal.style.display = 'block';

    var modalBody = document.getElementsByClassName('modal-body')[0];
    var modalHeader = document.getElementsByClassName('modal-header')[0];
    var modalFooter = document.getElementsByClassName('modal-footer')[0];

    if (showWarning == true) {
        modalHeader.innerHTML = '<span class="close">&times;</span><h2>SYSTEM ALERT<i class="fa fa-exclamation-triangle" style="font-size:40px; color:red;"></i></h2>';
        modalHeader.style.backgroundColor = '#ffcc00'; // yellow
        //modalHeader.style.color = 'black';
        modalFooter.style.backgroundColor = '#ffcc00';
        playSound('audioAlert1', true);
    }
    else {
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
    Count up Timer for when Guidance is engaged.
*/

function countUpTimer() {

    // Get todays date and time
    var now = new Date().getTime();
    // Find the elapsed time
    var elapsedTime = now - startDateTime.value;

    //engaged_timer = '00h 00m 00s';

    if (elapsedTime < 0)
    {
        //console.log('elapsedTime is negative');
        return;
    }

    // Time calculations for days, hours, minutes and seconds
    // var days = Math.floor(elapsedTime / (1000 * 60 * 60 * 24));
    var hours = Math.floor((elapsedTime % (1000 * 60 * 60 * 24)) / (1000 * 60 * 60));
    var minutes = Math.floor((elapsedTime % (1000 * 60 * 60)) / (1000 * 60));
    var seconds = Math.floor((elapsedTime % (1000 * 60)) / 1000);

    if (isGuidance.engaged == true) {
        engaged_timer = pad(hours, 2) + 'h '
            + pad(minutes, 2) + 'm ' + pad(seconds, 2) + 's ';
    }
    //console.log('engaged_timer: ' + engaged_timer);
}

/*
    For countUpTimer to format the time.
*/
function pad(num, size) {
    var s = "0000" + num;
    return s.substr(s.length - size);
}

/*
    Close the modal popup and reset variable as needed. 
*/
function closeModal(action) {

    var modal = document.getElementById('modalMessageBox');
    modal.style.display = 'none';

    isModalPopupShowing = false; //flag that modal popup has been closed.

    document.getElementById('audioAlert1').pause();
    document.getElementById('audioAlert2').pause();
    document.getElementById('audioAlert3').pause();
    document.getElementById('audioAlert4').pause();

    //alert('modal action:' + action);

    switch (action) {
       case 'RESTART':
            //Clear session variables except SystemReady (assumes interface manager still have driver's ready)
            isGuidance.remove();
            selectedRoute.remove();
            startDateTime.remove(); //resets the startDatetime
            clearInterval(timer); //stops the execution
            timer = null;
            engaged_timer = '00h 00m 00s';
             
            sessionStorage.removeItem('routePlanCoordinates');
            sessionStorage.removeItem('routeSpeedLimitDist');

            ready_counter = 0;
            ready_max_trial = 10;
            sound_counter = 0;
            sound_played_once = false;
            host_instructions = '';

            //clear sections
            document.getElementById('divCapabilitiesMessage').innerHTML = 'Please select a route.';
            clearTable('tblSecondA');

            CarmaJS.WidgetFramework.closeWidgets();

            // Get the element with id="defaultOpen" and click on it
            // This needs to be outside a funtion to work.
            document.getElementById('defaultOpen').click();

            //Update CAV buttons state back to Gray
            setCAVButtonState('DISABLED');

            //Evaluate next step
            evaluateNextStep();

            break;
        case 'LOGOUT':
            shutdown();
            break;
        default:
            //no action
            break;
    }
}

function shutdown()
{
    sessionStorage.clear();
    window.location.assign('scripts/killPlatform.php');
}
/*** Start: AUDIO ***/

/*
     It requires a “user gesture” to play or pause an audio or video element,
     which is enabled in Opera for Android, Chrome for Android, the default Android browser,
     Safari for iOS and probably other browsers.
     Firefox on Android is an exception, and works without this audio-fix.
*/
function loadAudioElements() {
    for (var i = 0; i < audioElements.length; i++) {
        audioElements[i].play();
        audioElements[i].pause();
    }
}

/*
    Play sound for specific alerts or notifications.
*/
function playSound(audioId, repeat) {
    var audioAlert1 = document.getElementById(audioId);
    audioAlert1.currentTime = 0;
    audioAlert1.play();

    if (repeat == false)
        return;

    //Repeat the sounds 5x max or until OK/logout page shows.
    setTimeout(function () {
        sound_counter++;
        if (sound_counter < sound_counter_max) {
            playSound(audioId, true);
        }
    }, 3000)
}

/*** End: AUDIO ***/
