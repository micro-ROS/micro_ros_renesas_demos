const rclnodejs = require('rclnodejs');
const { QoS } = rclnodejs;

const WebSocket = require('ws');
const wss = new WebSocket.Server({ port: 8080 });

rclnodejs.init().then(() => {
    const node = rclnodejs.createNode('graphic_motor_node');

    var connected = []
    node.createSubscription('std_msgs/msg/Float32', '/motor/speed', msg => {
      connected.forEach(e => {
        e.send(JSON.stringify({"speed":msg.data}))
      })
    });

    wss.on('connection', function connection(ws) {
      connected.push(ws)
    });

    rclnodejs.spin(node);
  });