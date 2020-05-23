/* //////////////// UDP Chat ////////////////
This is an example script showing how you can communicate with
the RCbenchmark APP from any UDP compatible source.

To test, simply install Packet Sender from:
https://packetsender.com/

Then configure the send_ip to your destination. If you use
Packet Sender in the same computer, write your local IP address
which will be displayed in the title bar of Packet Sender. With
Packet Sender, the send_port will be shown in the bottom right
as UDP:port#

The receive port is the port that packets should be send to in
order to receive back a reply. In Packet Sender, write this same
Port in the send section at the top.

In this example, you will receive a string containing all the sensor
values. You can copy/paste this string into a JSON parser to see it's
content (try http://jsonprettyprint.com/).

You can also send back an ESC signal to control your motor, like 1000.

Once you get this working with Packet Sender, you may move on
to your own application. For inspiration:

- Python UDP: https://wiki.python.org/moin/UdpCommunication
- MATLAB UDP: https://www.mathworks.com/help/instrument/udp.html

///////////// User defined variables //////////// */

var receive_port = 55047; // the listening port on this PC
var send_ip = "127.0.0.1"; // where to send the packet
var send_port = 64126;// on which port to send the packet
var samplesAvg = 50; // how many data samples to average before sending to TCP (helps to slow down communication rate)

///////////////// Beginning of the script //////////////////

// Setup continuous sensor read and motor control
function readSensor(){
    rcb.console.setVerbose(false);
    rcb.sensors.read(readDone, samplesAvg);
    rcb.console.setVerbose(true);
}
function readDone(result){
    // Send all the sensor data as a long JSON string
    var resultStr = JSON.stringify(result);
    var buffer = rcb.udp.str2ab(resultStr);
    rcb.udp.send(buffer);
    
    // Continue the loop
    readSensor();
}

/* Note the UDP functions expect arrayBuffer data type:
https://developer.mozilla.org/en-US/docs/Web/JavaScript/Typed_arrays */
rcb.udp.init(receive_port, send_ip, send_port, UDPInitialized);
rcb.udp.onReceive(UDPReceived); // Register callback event

function UDPInitialized(){
    var buffer = rcb.udp.str2ab("Hi from RCbenchmark script!");
    rcb.udp.send(buffer);
    readSensor();   // Start the sensor read loop until script is stopped.
}

function UDPReceived(arrayBuffer){
    var message = rcb.udp.ab2str(arrayBuffer);
    var throttle = Number(message);
    
    if(isNaN(throttle)){
        rcb.console.print("Received: " + message);
    }else{
        rcb.console.print("Setting ESC to: " + throttle);
        rcb.output.set("esc",throttle);   
    }
}
