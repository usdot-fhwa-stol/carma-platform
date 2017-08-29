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
    ros.connect('ws://192.168.32.133:9090');
}

/**
* Check System Alerts from Interface Manager
* TODO: Implement the user notification or modal popup.
**/
function checkSystemAlerts() {

    //alert("checkSystemAlerts");
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
            divCapabilitiesMessage.innerHTML = '<p> Sorry, there are no available routes, and cannot proceed without one. </p> <p> Please contact your System Admin.</p>';
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

            showPluginOptions();
        }
    });
}

/*
 Show user the registered plugins.
*/
function showPluginOptions() {

    //alert ('showPluginOptions');

    divCapabilitiesMessage.innerHTML = 'Please select one or more capabilities.';

    // Create a Service client with details of the service's name and service type.
    var getRegisteredPluginsClient = new ROSLIB.Service({
        ros: ros,
        name: '/plugins/getRegisteredPlugins',
        serviceType: 'cav_srvs/PluginList'
    });

    // Create a Service Request.
    // No arguments.
    var request = new ROSLIB.ServiceRequest({
    });

    // Call the service and get back the results in the callback.
    // The result is a ROSLIB.ServiceResponse object.
    getRegisteredPluginsClient.callService(request, function (result) {

        //alert('getRegisteredPluginsClient: begin');
        var pluginList = result.plugins;
        var divSubCapabilities = document.getElementById('divSubCapabilities');

        for (i = 0; i < pluginList.length; i++) {
            //alert('getRegisteredPluginsClient: for');
            var cbTitle = pluginList[i].name + ' ' + pluginList[i].versionId;
            var cbId = pluginList[i].name.replace(/\s/g,'_') + '&' + pluginList[i].versionId.replace(/\./g,'_') ;
            var isChecked = pluginList[i].activated;

            //alert (cbId);
            createCheckboxElement(divSubCapabilities, cbId, cbTitle, pluginList.length, 'groupPlugins', isChecked);
        }

        if (pluginList.length == 0) {
            //alert('getRegisteredPluginsClient: if');
            divCapabilitiesMessage.innerHTML = '<p> Sorry, there are no selection available, and cannot proceed without one. </p> <p> Please contact your System Admin.</p>';
        }

        //alert('getRegisteredPluginsClient: end');

    });
}

/*
 Activate the plugin based on user selection.
 rosservice call /plugins/activatePlugin '{header: auto, pluginName: DUMMY PLUGIN A, pluginVersion: v2.0.0, activated: True}'
*/
function activatePlugin(id) {

    var cbCapabilities = document.getElementById(id);
    var newStatus = cbCapabilities.checked; //Already set by browser to have registered checked value.

    alert ('activateplugin 1: ' + id + ', new: ' + newStatus);

    // Calling setActiveRoute service
    var activatePluginClient = new ROSLIB.Service({
        ros: ros,
        name: '/plugins/activatePlugin',
        serviceType: 'cav_srvs/PluginList'
    });

    // Then we create a Service Request.
    // No arguments.
    //            rosjava_test_msgs.TestHeader message =
    //                connectedNode.getTopicMessageFactory().newFromType(rosjava_test_msgs.TestHeader._TYPE);
    //            message.getHeader().setStamp(connectedNode.getCurrentTime());

    var splitValue = id.split('&');
    var name = splitValue[0].replace(/\_/g,' ');
    var version = splitValue[1].replace(/\_/g, '.');

    //alert ('activateplugin2: ' + name + ',' + version + ',' + newStatus);
    //alert(Date.now());
    var request = new ROSLIB.ServiceRequest({
        header: {
                  seq: 0
                  , stamp: Date.now()
                  , frame_id:''
        		},
        pluginName: name,
        pluginVersion: version,
        activated: newStatus
    });

    //alert ('activateplugin 3: ' + !newStatus);
   // alert('activatePluginClient.callService current status:' + cbCapabilities.checked );

    //IF it did not get into the callService below, need to set it back.
    cbCapabilities.checked = !newStatus;

    // Finally, we call the service and get back the results in the callback. The result
    // is a ROSLIB.ServiceResponse object.
    activatePluginClient.callService(request, function (result) {
        //alert('activatePluginClient.callService 1:' + result.newState + ', newstatus:' + newStatus);

        if (result.newState != newStatus) //NO_ROUTE
        {
            //alert ('if failed');
            divCapabilitiesMessage.innerHTML = '<p> Activating the capability failed, please try it again.</p>';
        }
        else {
            //alert ('else success:' + result.newState);
            var divSubCapabilities = document.getElementById('divSubCapabilities');
            divSubCapabilities.style.display = 'block';

            divCapabilitiesMessage.innerHTML = 'Please select one or more capabilities to activate.';
        }

        //Set to new state set by the PluginManager.
        cbCapabilities.checked = result.newState;

    });
}

window.onload = function () {
    connectToROS();
    checkSystemAlerts();

    //var divRouteOptions = document.createElement('divRoutes');

    showRouteOptions();

}