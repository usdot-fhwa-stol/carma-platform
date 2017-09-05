/***
 This file shall contain generic manipulation of html elements.
 Try not to mix ROS functionality here.
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
    newInput.id = radioId.toString();
    newInput.onclick = function () { setRoute(radioId.toString()) };

    var newLabel = document.createElement('label');
    newLabel.htmlFor = radioId.toString();
    newLabel.innerHTML = radioTitle;

    // Add the new elements to the container
    container.appendChild(newInput);
    container.appendChild(newLabel);

}

/*
* Adds a new checkbox onto the container.
* TODO: See if it can be combined with the one above.
*/
function createCheckboxElement (container, checkboxId, checkboxTitle, itemCount, groupName, isChecked){

    var newInput = document.createElement('input');
    newInput.type = 'checkbox';
    newInput.name = groupName;
    newInput.id = checkboxId.toString();
    newInput.checked = isChecked;
    newInput.onclick = function () { activatePlugin(checkboxId.toString()) };

    var newLabel = document.createElement('label');
    newLabel.htmlFor = checkboxId.toString();
    newLabel.innerHTML = checkboxTitle;

    // Add the new elements to the container
    container.appendChild(newInput);
    container.appendChild(newLabel);
}