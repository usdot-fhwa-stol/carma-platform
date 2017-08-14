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
	    name : '/system_alert',
	    messageType : 'cav_msgs/SystemAlert'
	  });

	  // Then we add a callback to be called every time a message is published on this topic.
	  listener.subscribe(function(message) {

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
	    var container =  document.getElementById('divLog');
	    var containerHeight = container.clientHeight;
	    var contentHeight = container.scrollHeight;
	    container.scrollTop = contentHeight - containerHeight;

	  });
