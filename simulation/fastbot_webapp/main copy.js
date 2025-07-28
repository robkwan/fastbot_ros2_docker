var app = new Vue({
    el: '#app',
    // storing the state of the page
    data: {
        connected: false,
        ros: null,
        logs: [],
        loading: false,
        rosbridge_address: 'wss://i-0db376d25bc33dd62.robotigniteacademy.com/99cc90d9-9b1d-498d-ad15-dadc4b0952ee/rosbridge/',
        port: '9090',
        // 3D stuff
        viewer: null,
        tfClient: null,
        urdfClient: null,
        position: { x: 0, y: 0, z: 0 },
        currentSpeed: 0,
        dragging: false,
        x: 'no',
        y: 'no',
        dragCircleStyle: {
            margin: '0px',
            top: '0px',
            left: '0px',
            display: 'none',
            width: '75px',
            height: '75px',
        },
        // joystick valules
        joystick: {
            vertical: 0,
            horizontal: 0,
        },
        // publisher
        pubInterval: null,
    },
    // helper methods to connect to ROS
    methods: {
        connect: function() {
            this.loading = true
            this.ros = new ROSLIB.Ros({
                url: this.rosbridge_address,
                groovyCompatibility: false
            })
            this.ros.on('connection', () => {
                this.logs.unshift((new Date()).toTimeString() + ' - Connected!')
                console.log("Connected event triggered");
                this.connected = true
                console.log("connected state:", this.connected);
                this.loading = false
                this.setupMap()
                this.setup3DViewer()
                this.setupCamera()
                this.pubInterval = setInterval(this.publish, 100)
                this.subscribeToOdometry() 
            }) 
            this.ros.on('error', (error) => {
                console.log('Error connecting to ROSBridge:', error)
                this.logs.unshift((new Date()).toTimeString() + ` - Error: ${error}`)
            })
            this.ros.on('close', () => {
                this.logs.unshift((new Date()).toTimeString() + ' - Disconnected!')
                console.log("Close got triggered!.")
                this.connected = false
                this.loading = false
                this.unset3DViewer()
                clearInterval(this.pubInterval)
            })
        },
        publish: function() {
            let topic = new ROSLIB.Topic({
                ros: this.ros,
                name: '/fastbot_1/cmd_vel',
                messageType: 'geometry_msgs/Twist'
            })
            let message = new ROSLIB.Message({
                linear: { x: this.joystick.vertical, y: 0, z: 0, },
                angular: { x: 0, y: 0, z: -(this.joystick.horizontal), },
            })
            topic.publish(message)
        },
        disconnect: function() {
            this.ros.close()
        },
       setupMap() {
            // Initialize map viewer
            this.mapViewer = new ROS2D.Viewer({
                divID: 'map',
                width: 420,
                height: 360
            });
            // Setup the map client
            this.mapGridClient = new ROS2D.OccupancyGridClient({
                ros: this.ros,
                rootObject: this.mapViewer.scene,
                continuous: true,
            });
            this.mapGridClient.on('change', () => {
                this.mapViewer.scaleToDimensions(this.mapGridClient.currentGrid.width, this.mapGridClient.currentGrid.height);
                this.mapViewer.shift(this.mapGridClient.currentGrid.pose.position.x, this.mapGridClient.currentGrid.pose.position.y);
            });
        },
        setup3DViewer() {
            this.viewer = new ROS3D.Viewer({
                background: '#cccccc',
                divID: 'div3DViewer',
                width: 400,
                height: 300,
                antialias: true,
                fixedFrame: 'fastbot_1_odom'
            })

            // Add a grid.
            this.viewer.addObject(new ROS3D.Grid({
                color:'#0181c4',
                cellSize: 0.5,
                num_cells: 20
            }))

            // Setup a client to listen to TFs.
            this.tfClient = new ROSLIB.TFClient({
                ros: this.ros,
                angularThres: 0.01,
                transThres: 0.01,
                rate: 10.0,
                fixedFrame: 'fastbot_1_base_link'
            })

            // Setup the URDF client.
            this.urdfClient = new ROS3D.UrdfClient({
                ros: this.ros,
                param: '/fastbot_1_robot_state_publisher:robot_description',
                tfClient: this.tfClient,
                // We use "path: location.origin + location.pathname"
                // instead of "path: window.location.href" to remove query params,
                // otherwise the assets fail to load
                path: location.origin + location.pathname,
                rootObject: this.viewer.scene,
                loader: ROS3D.COLLADA_LOADER_2
            })
        },
        unset3DViewer() {
            document.getElementById('div3DViewer').innerHTML = ''
        },
        setupCamera() {
            if (this.rosbridge_address && this.rosbridge_address.startsWith('wss://')) {
                let without_wss = this.rosbridge_address.split('wss://')[1];
                if (without_wss && without_wss.includes('/')) {
                    console.log(without_wss)
                    let domain = without_wss.split('/')[0] + '/' + without_wss.split('/')[1]
                    console.log(domain)
                    let host = domain + '/cameras';
                    // Ensure no port is appended to host
                    //host = host.replace(/:\d+/, ''); // Remove any port from the host if it exists
                    console.log(host);
                    let viewer = new MJPEGCANVAS.Viewer({
                        divID: 'divCamera',
                        host: host, // Use the host without port
                        width: 320,
                        height: 240,
                        topic: '/fastbot_1/camera/image_raw',
                        ssl: true,
                    });
                } else {
                    console.error("Invalid rosbridge_address format.");
                }
            } else {
                console.error("rosbridge_address must start with 'wss://'");
            }
        },
        subscribeToOdometry() {
            let topic = new ROSLIB.Topic({
                ros: this.ros,
                name: '/fastbot_1/odom',
                messageType: 'nav_msgs/Odometry'
            });
            topic.subscribe((message) => {
                this.position = message.pose.pose.position;
                this.currentSpeed = Math.sqrt(Math.pow(message.twist.twist.linear.x, 2) + Math.pow(message.twist.twist.linear.y, 2));
            });
        },
        startDrag() {
            this.dragging = true
            this.x = this.y = 0
        },
        stopDrag() {
            this.dragging = false
            this.x = this.y = 'no'
            this.dragCircleStyle.display = 'none'
            this.resetJoystickVals()
        },
        doDrag(event) {
            if (this.dragging) {
                this.x = event.offsetX
                this.y = event.offsetY
                let ref = document.getElementById('dragstartzone')
                this.dragCircleStyle.display = 'inline-block'

                let minTop = ref.offsetTop - parseInt(this.dragCircleStyle.height) / 2
                let maxTop = minTop + 200
                let top = this.y + minTop
                this.dragCircleStyle.top = `${top}px`

                let minLeft = ref.offsetLeft - parseInt(this.dragCircleStyle.width) / 2
                let maxLeft = minLeft + 200
                let left = this.x + minLeft
                this.dragCircleStyle.left = `${left}px`

                this.setJoystickVals()
            }
        },
        setJoystickVals() {
            this.joystick.vertical = -1 * ((this.y / 200) - 0.5)
            this.joystick.horizontal = +1 * ((this.x / 200) - 0.5)
        },
        resetJoystickVals() {
            this.joystick.vertical = 0
            this.joystick.horizontal = 0
        },
        goToWaypoint(waypoint) {
            let waypointPos = { x: 0, y: 0 }; // Define waypoints here
            switch (waypoint) {
                case 1: waypointPos = { x: 1, y: 1 }; break;
                case 2: waypointPos = { x: 2, y: 2 }; break;
                case 3: waypointPos = { x: 3, y: 3 }; break;
            }
            let topic = new ROSLIB.Topic({
                ros: this.ros,
                name: '/fastbot/move_base_simple/goal',
                messageType: 'geometry_msgs/PoseStamped'
            });
            let message = new ROSLIB.Message({
                header: {
                    frame_id: 'map',
                },
                pose: {
                    position: waypointPos,
                    orientation: { x: 0, y: 0, z: 0, w: 1 },
                }
            });
            topic.publish(message);
        },

    },
     mounted() {
        // page is ready
        window.addEventListener('mouseup', this.stopDrag)
    },
})