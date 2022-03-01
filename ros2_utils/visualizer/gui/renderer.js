// This file is required by the index.html file and will
// be executed in the renderer process for that window.
// No Node.js APIs are available in this process because
// `nodeIntegration` is turned off. Use `preload.js` to
// selectively enable features needed in the rendering
// process.

// const { join } = require('path')
// require(join(appRoot, 'rclnodejs'))


window.addEventListener('DOMContentLoaded', () => {
    const ws = require('electron').remote.getGlobal('sharedObject').ws

    speed_array = ['speed'].concat(new Array(100).fill(0))

    var chart = c3.generate({
        bindto: '#chart',
        point: {
            show: false
        },
        transition: {
            duration: 100
        },
        data: {
          columns: [
            speed_array
          ],
        },
        axis: {
            y: {
                max: 1200,
                min: -1200,
                label: {
                  text: 'Speed (rpm)',
                  position: 'outer-middle'
                }
            },
        }
    });

    ws.on('message', (data) => {
        data = JSON.parse(data)
        speed_array.push(data.speed)
        speed_array.splice(1,1)

        chart.load({
            columns: [
                speed_array
            ]
        });
    });
})
