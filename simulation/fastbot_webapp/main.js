var app = new Vue({
    el: '#app',
    // storing the state of the page
    data: {
        connected: false,
        ros: null,
        logs: [],
        loading: false,
        rosbridge_address: 'wss://i-003ae4e1d8556fb76.robotigniteacademy.com/4ee87bce-228a-477d-9742-336f5bd33e10/rosbridge/',
        port: '9090',
        // 3D stuff
        viewer: null,
        tfClient: null,
        urdfClient: null,
        position: { x: 0, y: 0, z: 0 },
        currentSpeed: 0,
        isAutonomous: false, // false = joystick mode, true = goal_pose nav mode
        cur_lin_x: 0,
        cur_ang_z: 0,
        lastPosition: null,
        lastYaw: null,
        lastTime: null,
        estimatedSpeed: {
            x: 0,
            z: 0,
        },
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
        controller_status: '',
    },
    // helper methods to connect to ROS
    methods: {
        connect: function () {
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
                //this.setupMap()
                this.setup3DViewer()
                //this.setupMapIn3DViewer()
                //this.setupCamera()
                this.pubInterval = setInterval(this.publish, 100)
                this.subscribeToOdometry()
                this.subscribeToControllerStatus()
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
        publish: function () {
            let topic = new ROSLIB.Topic({
                ros: this.ros,
                name: '/fastbot/cmd_vel',
                messageType: 'geometry_msgs/Twist'
            })
            let message = new ROSLIB.Message({
                linear: { x: this.joystick.vertical, y: 0, z: 0, },
                angular: { x: 0, y: 0, z: -(this.joystick.horizontal), },
            })
            topic.publish(message)
        },
        disconnect: function () {
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
        setupMapIn3DViewer() {
            const gridClient = new ROS3D.OccupancyGridClient({
                ros: this.ros,
                rootObject: this.viewer.scene,  // ✅ Use 3D viewer's scene
                continuous: true
            });

            //gridClient.on('change', () => {
            //    this.viewer.scaleToDimensions(
            //        gridClient.currentGrid.width,
            //        gridClient.currentGrid.height
            //    );
            //    this.viewer.shift(
            //        gridClient.currentGrid.pose.position.x,
            //        gridClient.currentGrid.pose.position.y
            //    );
            //});
        },
        setup3DViewer() {
            this.viewer = new ROS3D.Viewer({
                background: '#cccccc',
                divID: 'div3DViewer',
                width: 400,
                height: 300,
                antialias: true,
                fixedFrame: 'fastbot_odom' //'map' //'fastbot_1_odom'
            })

            // Add a grid.
            this.viewer.addObject(new ROS3D.Grid({
                color: '#0181c4',
                cellSize: 0.5,
                num_cells: 20
            }))

            this.gridClient = new ROS3D.OccupancyGridClient({
                ros: this.ros,
                rootObject: this.viewer.scene,
                continuous: true,
            });

            this.viewer.addObject(new ROS3D.Axes({
                tfClient: this.tfClient,
                frameId: 'fastbot_base_link',
                size: 0.5
            }));

            // Setup a client to listen to TFs.
            this.tfClient = new ROSLIB.TFClient({
                ros: this.ros,
                angularThres: 0.01,
                transThres: 0.01,
                rate: 10.0,
                fixedFrame: 'fastbot_base_link' //'map' //'fastbot_1_base_link'
            })

            // Setup the URDF client.
            this.urdfClient = new ROS3D.UrdfClient({
                ros: this.ros,
                param: '/fastbot_robot_state_publisher:robot_description',
                tfClient: this.tfClient,
                // We use "path: location.origin + location.pathname"
                // instead of "path: window.location.href" to remove query params,
                // otherwise the assets fail to load
                path: location.origin + location.pathname,
                rootObject: this.viewer.scene,
                loader: ROS3D.COLLADA_LOADER_2
            })

            this.viewer.camera.position.set(8, -12, 10);  // basic Z-height
            this.viewer.camera.lookAt(new THREE.Vector3(0, 0, 0));

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
                        topic: '/fastbot_camera/image_raw',
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
                name: '/fastbot/odom',
                messageType: 'nav_msgs/msg/Odometry'
            });
            topic.subscribe((message) => {
                console.log("RAW ODOM MESSAGE RECEIVED:", message);
                const now = Date.now(); // milliseconds
                this.pose = message.pose.pose;
                this.position = message.pose.pose.position;
                console.log(this.position)
                const quat = message.pose.pose.orientation;

                // Convert quaternion to yaw (Z-axis rotation)
                const siny_cosp = 2 * (quat.w * quat.z + quat.x * quat.y);
                const cosy_cosp = 1 - 2 * (quat.y * quat.y + quat.z * quat.z);
                const yaw = Math.atan2(siny_cosp, cosy_cosp);  // in radians

                //this.currentSpeed = Math.sqrt(Math.pow(message.twist.twist.linear.x, 2) + Math.pow(message.twist.twist.linear.y, 2));
                //this.cur_lin_x = message.twist.twist.linear.x;
                //this.cur_ang_z = message.twist.twist.angular.z;
                if (this.lastPosition && this.lastTime) {
                    const dx = this.position.x - this.lastPosition.x;
                    const dy = this.position.y - this.lastPosition.y;

                    let dyaw = yaw - this.lastYaw;

                    // Normalize to [-PI, PI]
                    if (dyaw > Math.PI) dyaw -= 2 * Math.PI;
                    if (dyaw < -Math.PI) dyaw += 2 * Math.PI;

                    const dt = (now - this.lastTime) / 1000.0; // seconds
                    if (dt > 0) {
                        this.estimatedSpeed.x = Math.sqrt(dx * dx + dy * dy) / dt;
                        this.estimatedSpeed.z = dyaw / dt;  // rad/s
                    }
                }

                this.lastPosition = { x: this.position.x, y: this.position.y };
                this.lastYaw = yaw;
                this.lastTime = now;

                // Update 3D model position based on odometry data
                this.update3DModelPosition(this.pose);

            });
        },
        update3DModelPosition(pose) {
            // Ensure the model is updated with respect to the grid
            const gridSize = 1; // Change this to match your grid size

            if (this.urdfClient && this.urdfClient.rootObject) {
                let model = this.urdfClient.rootObject; // Use the rootObject directly

                // Snap the position to the nearest grid point
                model.position.x = Math.round(pose.position.x / gridSize) * gridSize;
                model.position.y = Math.round(pose.position.y / gridSize) * gridSize;
                model.position.z = pose.position.z; // Keep the z position as is if not affected by the grid

                // Set the orientation using quaternion
                model.quaternion.set(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
            }
        },
        startDrag() {
            this.dragging = true
            this.x = this.y = 0
            this.controller_status = ''
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

                // Optionally publish velocity commands
                this.publish(); // Send velocity commands based on joystick input

                // Define a speed factor based on your grid size
                //const speedFactor = 0.1; // Adjust as needed for your grid scale

                // Calculate new position based on joystick input
                //let moveX = this.joystick.horizontal * speedFactor;
                //let moveY = this.joystick.vertical * speedFactor;

                // Update the model's position based on movement
                //if (this.urdfClient && this.urdfClient.rootObject) {
                //    let model = this.urdfClient.rootObject;
                //    model.position.x += moveX;
                //    model.position.y += moveY;

                // Optionally ensure the model stays within grid boundaries
                //    model.position.x = Math.max(minX, Math.min(maxX, model.position.x));
                //    model.position.y = Math.max(minY, Math.min(maxY, model.position.y));
                //}

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
                case 1: waypointPos = { x: 4.27, y: 1.94 }; break;
                case 2: waypointPos = { x: -0.69, y: 1.87 }; break;
                case 3: waypointPos = { x: 1.99, y: 5.71 }; break;
            }
            let topic = new ROSLIB.Topic({
                ros: this.ros,
                name: '/goal_pose',
                messageType: 'geometry_msgs/msg/PoseStamped'
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
            this.isAutonomous = true;
        },
        subscribeToControllerStatus() {
            const transitionTopic = new ROSLIB.Topic({
                ros: this.ros,
                name: '/controller_server/transition_event',
                messageType: 'lifecycle_msgs/msg/TransitionEvent'
            });

            transitionTopic.subscribe((msg) => {
                const startState = msg.start_state.label;
                const goalState = msg.goal_state.label;

                const statusMsg = `Controller state changed from '${startState}' to '${goalState}'`;
                console.log('[Transition]', statusMsg);
                this.controller_status = statusMsg;

                // Optional: update 'reached' flag heuristically
                if (goalState === 'inactive') {
                    this.reached = false;
                } else if (goalState === 'finalized') {
                    this.reached = true;
                }
            });

            // Optional: also watch /rosout for logs if needed
            const rosout = new ROSLIB.Topic({
                ros: this.ros,
                name: '/rosout',
                messageType: 'rcl_interfaces/msg/Log'
            });

            rosout.subscribe((msg) => {
                if (msg.name && msg.name.includes('controller_server')) {
                    console.log('[ROSOUT][Controller]', msg.msg);
                    this.controller_status = msg.msg;

                    if (msg.msg.includes('Reached') || msg.msg.includes('SUCCEEDED')) {
                        this.reached = true;
                        this.isAutonomous = false;
                    } else if (msg.msg.includes('Aborted') || msg.msg.includes('FAILED')) {
                        this.reached = false;
                    }
                }
            });
        }
    },
    mounted() {
        // page is ready
        window.addEventListener('mouseup', this.stopDrag)
    },
})