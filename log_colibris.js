var SignalKClient = require('@signalk/client').Client;
var signalk = new SignalKClient;

var connection;

var host = '10.8.0.17:3000';
var subscribe = '{ "context": "vessels.self", "subscribe": [{ "path": "environment.depth.belowTransducer"},{"path": "navigation.attitude"}]}';
//var subscribe = '{ "context": "vessels.self", "subscribe": [{ "path": "environment.depth.belowTransducer"}]}';

var thisCallback = function(msg) {
  if (msg.updates) {
    msg.updates.forEach(function(update) {
      update.values.forEach(function(value) {
            console.log(value.path,value.value);
        })
      })
  }
};

var onConnect = function(connection) {
  connection.send(subscribe);
}

function connectDelta(host, thisCallback, onConnect, onDisconnect) {
  console.log("Connecting to " + host);

  connection = signalk.connectDelta(host, thisCallback,
    function(skConnection) {
      onConnect(skConnection);
      console.log('Connected');
    },

    function(skConnection) {
      skConnection.close();
      console.log('Disconnected');
    },

    function(error) {
      console.log(error)
    },
    function close(){},
    'none'
  );
}

function wait(ms)
{
var d = new Date();
var d2 = null;
do { d2 = new Date(); }
while(d2-d < ms);
}

connectDelta(host, thisCallback, onConnect);
