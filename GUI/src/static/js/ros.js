let rosConnected = false;

// Function to fetch the host IP
async function getHostIP() {
  try {
    const response = await fetch('/get-ip');
    if (!response.ok) {
      throw new Error('Network response was not ok');
    }
    const data = await response.json();
    console.log('Host IP:', data.ip);
    return data.ip;
  } catch (error) {
    console.error('Error fetching IP:', error);
  }
}

// Function to set up ROS connection and topics
async function setupROS() {
  const ip = await getHostIP();
  const ros = new ROSLIB.Ros({
    url: 'ws://' + ip + ':9090',
  });

  console.log('Connecting to websocket server at ws://' + ip + ':9090');

  // ROS connection event handlers
  ros.on('connection', function () {
    console.log('Connected to websocket server.');
    diagnosticInput.value += 'Connected to websocket server.\n';
    rosConnected = true;
  });

  ros.on('error', function (error) {
    console.log('Error connecting to websocket server: ', error);
    diagnosticInput.value += 'Error connecting to websocket server: ' + error + '\n';
  });

  ros.on('close', function () {
    console.log('Connection to websocket server closed.');
    diagnosticInput.value += 'Connection to websocket server closed.\n';
  });

  // Reconnection logic
  const reconnectIntervalId = setInterval(function () {
    if (!rosConnected) {
      console.log('Trying to reconnect to websocket server.');
      diagnosticInput.value += 'Trying to reconnect to websocket server.\n';
      ros.connect('ws://' + ip + ':9090');
    }

    if (rosConnected) {
      clearInterval(reconnectIntervalId);
    }
  }, 1000);

  // Initialize publishers and subscribers
  initializeSubscribers(ros);
  // initializePublishers(ros);
}

// Function to initialize all subscribers
function initializeSubscribers(ros) {
  // Compass data subscriber
  const compassDataListener = new ROSLIB.Topic({
    ros: ros,
    name: '/compass_data_topic',
    messageType: 'std_msgs/Float64',
  });

  compassDataListener.subscribe(function (message) {
    compass = message.data;
  });

  // Listener for string messages
  const listener = new ROSLIB.Topic({
    ros: ros,
    name: '/listener',
    messageType: 'std_msgs/String',
  });

  listener.subscribe(function (message) {
    diagnosticInput.value += 'Received message on ' + listener.name + ': ' + message.data + '\n';
  });

  // Voltage subscriber
  const voltageListener = new ROSLIB.Topic({
    ros: ros,
    name: '/voltage',
    messageType: 'std_msgs/Float32',
  });

  voltageListener.subscribe(function (message) {
    curr_battery = message.data;
  });

  // Ampere subscriber
  const ampereListener = new ROSLIB.Topic({
    ros: ros,
    name: '/ampere',
    messageType: 'std_msgs/Float32',
  });

  ampereListener.subscribe(function (message) {
    curr_ampere = message.data;
  });

  // X and Y coordinates subscribers
  const xCoordinatesListener = new ROSLIB.Topic({
    ros: ros,
    name: '/xcoordinates',
    messageType: 'std_msgs/Float32',
  });

  xCoordinatesListener.subscribe(function (message) {
    xcoord = message.data;
  });

  const yCoordinatesListener = new ROSLIB.Topic({
    ros: ros,
    name: '/ycoordinates',
    messageType: 'std_msgs/Float32',
  });

  yCoordinatesListener.subscribe(function (message) {
    ycoord = message.data;
  });

  // Velocity subscribers
  const xVelocityListener = new ROSLIB.Topic({
    ros: ros,
    name: '/xvelocity',
    messageType: 'std_msgs/Float32',
  });

  xVelocityListener.subscribe(function (message) {
    xvelocity = message.data;
  });

  const yVelocityListener = new ROSLIB.Topic({
    ros: ros,
    name: '/yvelocity',
    messageType: 'std_msgs/Float32',
  });

  yVelocityListener.subscribe(function (message) {
    yvelocity = message.data;
  });

  // Acceleration subscriber
  const accelListener = new ROSLIB.Topic({
    ros: ros,
    name: '/acceleration',
    messageType: 'std_msgs/Float32',
  });

  accelListener.subscribe(function (message) {
    accel = message.data;
  });

  // Distance subscriber
  const distanceListener = new ROSLIB.Topic({
    ros: ros,
    name: '/distance',
    messageType: 'std_msgs/Float32',
  });

  distanceListener.subscribe(function (message) {
    distance = message.data;
  });

  // Temperature subscriber
  const tempListener = new ROSLIB.Topic({
    ros: ros,
    name: '/temp',
    messageType: 'std_msgs/Float32',
  });

  tempListener.subscribe(function (message) {
    for (let i = 0; i < 7; i++) {
      tempBoxes[i].innerHTML = message.data;
    }
  });
}

// Function to initialize all publishers
function initializePublishers(ros) {
  // Compass data publisher
  const compassDataPublisher = new ROSLIB.Topic({
    ros: ros,
    name: '/compass_data_topic',
    messageType: 'std_msgs/Float64',
  });

  setInterval(function () {
    compassDataPublisher.publish(
      new ROSLIB.Message({
        data: Math.random() * 360, // Simulating compass data in degrees
      })
    );
  }, 1000);

  // Listener publisher
  const listenerPublisher = new ROSLIB.Topic({
    ros: ros,
    name: '/listener',
    messageType: 'std_msgs/String',
  });

  setInterval(function () {
    if (storedText == '') {
      return;
    }
    listenerPublisher.publish(
      new ROSLIB.Message({
        data: 'Commencing: ' + storedText,
      })
    );
  }, 5000);

  // Voltage publisher
  const voltagePublisher = new ROSLIB.Topic({
    ros: ros,
    name: '/voltage',
    messageType: 'std_msgs/Float32',
  });

  setInterval(function () {
    voltagePublisher.publish(
      new ROSLIB.Message({
        data: Math.round(Math.random() * 10, 2),
      })
    );
  }, 1000);

  // Ampere publisher
  const amperePublisher = new ROSLIB.Topic({
    ros: ros,
    name: '/ampere',
    messageType: 'std_msgs/Float32',
  });

  setInterval(function () {
    amperePublisher.publish(
      new ROSLIB.Message({
        data: Math.round(Math.random() * 10, 2),
      })
    );
  }, 1000);

  // X and Y coordinates publishers
  const xCoordinatesPublisher = new ROSLIB.Topic({
    ros: ros,
    name: '/xcoordinates',
    messageType: 'std_msgs/Float32',
  });

  setInterval(function () {
    xCoordinatesPublisher.publish(
      new ROSLIB.Message({
        data: Math.round(Math.random() * 10, 2),
      })
    );
  }, 1000);

  const yCoordinatesPublisher = new ROSLIB.Topic({
    ros: ros,
    name: '/ycoordinates',
    messageType: 'std_msgs/Float32',
  });

  setInterval(function () {
    yCoordinatesPublisher.publish(
      new ROSLIB.Message({
        data: Math.round(Math.random() * 10, 2),
      })
    );
  }, 1000);

  // Velocity publishers
  const xVelocityPublisher = new ROSLIB.Topic({
    ros: ros,
    name: '/xvelocity',
    messageType: 'std_msgs/Float32',
  });

  setInterval(function () {
    xVelocityPublisher.publish(
      new ROSLIB.Message({
        data: Math.round(Math.random() * 10, 2),
      })
    );
  }, 1000);

  const yVelocityPublisher = new ROSLIB.Topic({
    ros: ros,
    name: '/yvelocity',
    messageType: 'std_msgs/Float32',
  });

  setInterval(function () {
    yVelocityPublisher.publish(
      new ROSLIB.Message({
        data: Math.round(Math.random() * 10, 2),
      })
    );
  }, 1000);

  // Acceleration publisher
  const accelPublisher = new ROSLIB.Topic({
    ros: ros,
    name: '/acceleration',
    messageType: 'std_msgs/Float32',
  });

  setInterval(function () {
    accelPublisher.publish(
      new ROSLIB.Message({
        data: Math.round(Math.random() * 10, 2),
      })
    );
  }, 1000);

  // Distance publisher
  const distancePublisher = new ROSLIB.Topic({
    ros: ros,
    name: '/distance',
    messageType: 'std_msgs/Float32',
  });

  setInterval(function () {
    distancePublisher.publish(
      new ROSLIB.Message({
        data: Math.round(Math.random() * 10, 2),
      })
    );
  }, 1000);

  // Temperature publisher
  const tempPublisher = new ROSLIB.Topic({
    ros: ros,
    name: '/temp',
    messageType: 'std_msgs/Float32',
  });

  setInterval(function () {
    tempPublisher.publish(
      new ROSLIB.Message({
        data: Math.round(Math.random() * 10, 2),
      })
    );
  }, 1000);
}

// Initialize ROS setup
setupROS();