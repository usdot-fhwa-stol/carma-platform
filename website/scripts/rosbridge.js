	  // Connecting to ROS
	  // -----------------
	  var ros = new ROSLIB.Ros();

	  // If there is an error on the backend, an 'error' emit will be emitted.
	  ros.on('error', function(error) {
	    document.getElementById('connecting').style.display = 'none';
	    document.getElementById('connected').style.display = 'none';
	    document.getElementById('closed').style.display = 'none';
	    document.getElementById('error').style.display = 'inline';
	    console.log(error);
	  });

	  // Find out exactly when we made a connection.
	  ros.on('connection', function() {
	    console.log('Connection made!');
	    document.getElementById('connecting').style.display = 'none';
	    document.getElementById('error').style.display = 'none';
	    document.getElementById('closed').style.display = 'none';
	    document.getElementById('connected').style.display = 'inline';
	  });

	  ros.on('close', function() {
	    console.log('Connection closed.');
	    document.getElementById('connecting').style.display = 'none';
	    document.getElementById('connected').style.display = 'none';
	    document.getElementById('closed').style.display = 'inline';
	  });

	  // Create a connection to the rosbridge WebSocket server.
	  ros.connect('ws://192.XXX.XX.XXX:9090');

	  //Subscribing to a Topic
	  //----------------------

	  // Like when publishing a topic, we first create a Topic object with details of the topic's name
	  // and message type. Note that we can call publish or subscribe on the same topic object.
	  var listener = new ROSLIB.Topic({
	    ros : ros,
	    name : '/template',
	    messageType : 'std_msgs/String'
	  });

	  // Then we add a callback to be called every time a message is published on this topic.
	  listener.subscribe(function(message) {
	    console.log('Received message on ' + listener.name + ': ' + message.data);
	    document.getElementById('divLog').innerHTML += '<br/> Received message on ' + listener.name + ': ' + message.data;

	    // If desired, we can unsubscribe from the topic as well.
	    //listener.unsubscribe();
	  });
