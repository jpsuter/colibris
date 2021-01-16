var request = require('request');
var sleep = require('sleep');

var Database_URL = 'localhost';

var options = {
	url: 'http://10.8.0.17:3000/signalk/v1/api/vessels/self/',
	method: 'GET',
	forever: false,
	json: true
}

var mysql = require('mysql'); //https://www.npmjs.com/package/mysql

var connection = mysql.createConnection({
	host: Database_URL,
	user: "jps",
	password: "test1",
	database: "Colibris"
});

connection.connect(function(err) {
	if (err) throw err;
	console.log("Database Connected!");
});

function insert_message(message_str) {
	var sql = "INSERT INTO ?? (??) VALUES (?)";
	var params = ['JrnlColibris', 'message', message_str];
	sql = mysql.format(sql, params);	
	
	connection.query(sql, function (error, results) {
		if (error) throw error;
		console.log("Message added: " + message_str);
	}); 
};	

function callback(err, res, body) {
          if (err) { return console.log(err); }
          var message_str = JSON.stringify(body);
          insert_message(message_str);
          console.log("insert OK");
};

request(options, callback) ;
