/***
 This file shall contain ROS relate function calls.
 Try not to move html element manipulation to controls.js.

 TODO: Store user selections across postbacks using javascript.
****/

var ros = new ROSLIB.Ros();
var divCapabilitiesMessage = document.getElementById('divCapabilitiesMessage');

/*
* Connection to ROS
*/
function connectToROS() {
    // If there is an error on the backend, an 'error' emit will be emitted.
    ros.on('error', function (error) {
        document.getElementById('connecting').style.display = 'none';
        document.getElementById('connected').style.display = 'none';
        document.getElementById('closed').style.display = 'none';
        document.getElementById('error').style.display = 'inline';
        console.log(error);
    });

    // Find out exactly when we made a connection.
    ros.on('connection', function () {
        console.log('Connection made!');
        document.getElementById('connecting').style.display = 'none';
        document.getElementById('error').style.display = 'none';
        document.getElementById('closed').style.display = 'none';
        document.getElementById('connected').style.display = 'inline';
    });

    ros.on('close', function () {
        console.log('Connection closed.');
        document.getElementById('connecting').style.display = 'none';
        document.getElementById('connected').style.display = 'none';
        document.getElementById('closed').style.display = 'inline';
    });

    // TODO: Update IP
    // Create a connection to the rosbridge WebSocket server.
    ros.connect('ws://192.168.32.132:9090');
}

/**
* Check System Alerts from Interface Manager
* TODO: Implement the user notification or modal popup.
**/
function checkSystemAlerts() {
    //Subscribing to a Topic
    //----------------------

    // Like when publishing a topic, we first create a Topic object with details of the topic's name
    // and message type. Note that we can call publish or subscribe on the same topic object.
    var listener = new ROSLIB.Topic({
        ros: ros,
        name: '/system_alert',
        messageType: 'cav_msgs/SystemAlert'
    });

    // Then we add a callback to be called every time a message is published on this topic.
    listener.subscribe(function (message) {

        var messageTypeFullDescription = 'NA';

        switch (message.type) {
            case 1:
                messageTypeFullDescription = 'Take caution! ';
                break;
            case 2:
                messageTypeFullDescription = 'I have a warning! ';
                break;
            case 3:
                messageTypeFullDescription = 'I am FATAL! ';
                break;
            case 4:
                messageTypeFullDescription = 'I am NOT Ready! ';
                break;
            case 5:
                messageTypeFullDescription = 'I am Ready! ';
                break;
            default:
                messageTypeFullDescription = 'I am NOT Ready! ';
        }

        document.getElementById('divLog').innerHTML += '<br/> ' + message.description + '; ' + messageTypeFullDescription;

        //Make sure message list is scrolled to the bottom
        var container = document.getElementById('divLog');
        var containerHeight = container.clientHeight;
        var contentHeight = container.scrollHeight;
        container.scrollTop = contentHeight - containerHeight;

    });
}

/*
 Show user the available route options.
*/
function showRouteOptions() {

    divCapabilitiesMessage.innerHTML = 'Please select a route.';

    // Create a Service client with details of the service's name and service type.
    var getAvailableRoutesClient = new ROSLIB.Service({
        ros: ros,
        name: '/get_available_routes', // '/vehicle_environment/route/get_available_routes',
        serviceType: 'cav_srvs/GetAvailableRoutes'
    });

    // Create a Service Request.
    // No arguments.
    var request = new ROSLIB.ServiceRequest({

    });
    // Call the service and get back the results in the callback.
    // The result is a ROSLIB.ServiceResponse object.
    getAvailableRoutesClient.callService(request, function (result) {

        var myRoutes = result.availableRoutes;
        var divRoutes = document.getElementById('divRoutes');

        for (i = 0; i < myRoutes.length; i++) {
            createRadioElement(divRoutes, myRoutes[i].routeID, myRoutes[i].routeName, myRoutes.length, 'groupRoutes');
        }

        if (myRoutes.length == 0) {
            divCapabilitiesMessage.innerHTML = '<p> Sorry, there are no available routes, and cannot proceed without one. </p> <p> Please contact the System Admin.</p>';
        }

    });
}

/*
 Set the route once based on user selection.
*/
function setRoute(id) {

    // Calling setActiveRoute service
    var setActiveRouteClient = new ROSLIB.Service({
        ros: ros,
        name: '/set_active_route', // '/vehicle_environment/route/get_available_routes',
        serviceType: 'cav_srvs/SetActiveRoute'
    });

    // Then we create a Service Request.
    // No arguments.
    var request = new ROSLIB.ServiceRequest({
        routeID: id.toString()
    });

    // Finally, we call the service and get back the results in the callback. The result
    // is a ROSLIB.ServiceResponse object.
    setActiveRouteClient.callService(request, function (result) {

        if (result.errorStatus == 1) //NO_ROUTE
        {
            divCapabilitiesMessage.innerHTML = '<p> Activating the route failed, please try it again.</p>';

            //Allow user to select it again.
            var rbRoute = document.getElementById(id.toString());
            rbRoute.checked = false;
        }
        else {
            var divRoutes = document.getElementById('divRoutes');
            divRoutes.style.display = 'none';

            var divSubCapabilities = document.getElementById('divSubCapabilities');
            divSubCapabilities.style.display = 'block';

            divCapabilitiesMessage.innerHTML = 'Please select one or more capabilities to activate.';
        }
    });
}

window.onload = function () {
    connectToROS();
    showRouteOptions();
    checkSystemAlerts();
}