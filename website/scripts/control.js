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
        //cell1.setAttribute("class","col-style2a");
        //cell2.setAttribute("class","col-style2b");
        cell1.className = 'col-style2a';
        cell2.className = 'col-style2b';
        cell2.id = cellId;
    }
}