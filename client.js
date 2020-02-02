/**
 * Establishes connection to a ROS master via websocket.
 **/
function KnowrobClient(options){
    var that = this;
    // Object that holds user information
    this.flask_user = options.flask_user;
    // ROS handle
    this.ros = undefined;
    // URL for ROS server
    var rosURL = options.ros_url || 'ws://localhost:9090';
    // Use rosauth?
    var authentication  = options.authentication === '' ? true : options.authentication === 'True';
    // URL for rosauth token retrieval
    var authURL  = options.auth_url || '/wsauth/v1.0/by_session';
    // global jsonprolog handle
    var prolog;
    
    this.pageOverlayDisabled = false;
    // true iff connection to ROS master is established
    this.isConnected = false;
    // true iff json_prolog is connected
    this.isPrologConnected = false;
    // true iff registerNodes was called before
    this.isRegistered = false;
    // Prefix for mesh GET URL's
    var meshPath  = options.meshPath || '/';

    // sprite markers and render events
    var sprites = [];
    var render_event;

    // The selected marker object or undefined
    this.selectedMarker = undefined;
    
    // ROS messages
    var tfClient = undefined;
    this.markerArrayClient = undefined;
    var designatorClient = undefined;
    var imageClient = undefined;
    var cameraPoseClient = undefined;
    this.snapshotTopic = undefined;
    
    this.nodesRegistered = false;
    
    this.init = function() {
        // Connect to ROS.
        that.connect();
        
        that.createOverlay();
        that.showPageOverlay("Loading Knowledge Base");
      
        setInterval(containerRefresh, 570000);
        containerRefresh();
        render_event = new CustomEvent('render', {'camera': null});
    };
    
    function containerRefresh() {
        $.ajax({
            url: '/api/v1.0/refresh_by_session',
            type: "GET",
            contentType: "application/json",
            dataType: "json"
        });
    };

    this.connect = function () {
      if(that.ros) return;
      that.ros = new ROSLIB.Ros({url : rosURL});
      that.ros.on('connection', function() {
          that.isConnected = true;
          console.log('Connected to websocket server.');
          if (authentication) {
              // Acquire auth token for current user and authenticate, then call registerNodes
              that.authenticate(authURL, that.registerNodes);
          } else {
              // No authentication requested, call registerNodes directly
              that.registerNodes();
              that.waitForJsonProlog();
          }
      });
      that.ros.on('close', function() {
          console.log('Connection was closed.');
          that.showPageOverlay("Connection was closed, reconnecting...");
          that.ros = undefined;
          that.isRegistered = false;
          setTimeout(that.connect, 500);
      });
      that.ros.on('error', function(error) {
          console.log('Error connecting to websocket server: ', error);
          that.showPageOverlay("Connection error, reconnecting...");
          if(that.ros) that.ros.close();
          that.ros = undefined;
          that.isRegistered = false;
          setTimeout(that.connect, 500);
      });
    };

    this.authenticate = function (authurl, then) {
        console.log("Acquiring auth token");
        // Call wsauth api to acquire auth token by existing user login session
        $.ajax({
            url: authurl,
            type: "GET",
            contentType: "application/json",
            dataType: "json"
        }).done( function (request) {
            if(!that.ros) {
                console.warn("Lost connection to ROS master.");
                return;
            }
            console.log("Sending auth token");
            that.ros.authenticate(request.mac,
                             request.client,
                             request.dest,
                             request.rand,
                             request.t,
                             request.level,
                             request.end);
            that.waitForJsonProlog();
            
            // If a callback function was specified, call it in the context of Knowrob class (that)
            if(then) {
                then.call(that);
            }
        });
    };
    
    this.registerNodes = function () {
      if(that.isRegistered) return;
      that.isRegistered = true;
      
      // Setup publisher that sends a dummy message in order to keep alive the socket connection
      {
          var interval = options.interval || 30000;
          // The topic dedicated to keep alive messages
          var keepAliveTopic = new ROSLIB.Topic({ ros : that.ros, name : '/keep_alive', messageType : 'std_msgs/Bool' });
          // A dummy message for the topic
          var keepAliveMsg = new ROSLIB.Message({ data : true });
          // Function that publishes the keep alive message
          var ping = function() { keepAliveTopic.publish(keepAliveMsg); };
          // Call ping at regular intervals.
          setInterval(ping, interval);
      };
    
      // topic used for publishing canvas snapshots
      that.snapshotTopic = new ROSLIB.Topic({
        ros : that.ros,
        name : '/openease/video/frame',
        messageType : 'sensor_msgs/Image'
      });
      
      // Setup a client to listen to TFs.
      tfClient = new ROSLIB.TFClient({
        ros : that.ros,
        angularThres : 0.01,
        transThres : 0.01,
        rate : 10.0,
        fixedFrame : '/my_frame'
      });

      // Setup the marker array client.
      that.markerArrayClient = new EASE.MarkerArrayClient({
        ros : that.ros,
        tfClient : tfClient,
        topic : '/visualization_marker_array',
        canvas : that.canvas,
        path : meshPath
      });

      // Setup the designator message client.
      // TODO: replace by something more general
      designatorClient = new ROSLIB.Topic({
        ros : that.ros,
        name : '/logged_designators',
        messageType : 'designator_integration_msgs/Designator'
      });
      designatorClient.subscribe(function(message) {
        if(message.description.length==0) {
          console.warn("Ignoring empty designator.");
        }
        else {
          var desig_js = parse_designator(message.description);
          var html = format_designator(message.description);
          if(that.on_designator_received)
            that.on_designator_received(html);
        }
      });
        
      // Setup the image message client.
      imageClient = new ROSLIB.Topic({
        ros : that.ros,
        name : '/logged_images',
        messageType : 'std_msgs/String'
      });
      imageClient.subscribe(function(message) {
          var ext = message.data.substr(message.data.lastIndexOf('.') + 1).toLowerCase();
          var url = message.data;
          if(!url.startsWith("/knowrob/")) {
               if(url.startsWith("/home/ros/user_data"))  url = '/user_data/'+url.replace("/home/ros/user_data/", "");
               else url = '/knowrob/knowrob_data/'+url;
          }
          var imageHeight, imageWidth;
          var html = '';
          if(ext=='jpg' || ext =='png') {
              html += '<div class="image_view">';
              html += '<img id="mjpeg_image" class="picture" src="'+url+'" width="300" height="240"/>';
              html += '</div>';
              
              imageHeight = function(mjpeg_image) { return mjpeg_image.height; };
              imageWidth  = function(mjpeg_image) { return mjpeg_image.width; };
          }
          else if(ext =='ogg' || ext =='ogv' || ext =='mp4' || ext =='mov') {
              html += '<div class="image_view">';
              html += '  <video id="mjpeg_image" controls autoplay loop>';
              html += '    <source src="'+url+'" ';
              if(ext =='ogg' || ext =='ogv') html += 'type="video/ogg" ';
              else if(ext =='mp4') html += 'type="video/mp4" ';
              html += '/>';
              html += 'Your browser does not support the video tag.';
              html += '</video></div>';
              
              imageHeight = function(mjpeg_image) { return mjpeg_image.videoHeight; };
              imageWidth  = function(mjpeg_image) { return mjpeg_image.videoWidth; };
          }
          else {
              console.warn("Unknown data format on /logged_images topic: " + message.data);
          }
          if(html.length>0 && that.on_image_received) {
              that.on_image_received(html, imageWidth, imageHeight);
          }
      });

      cameraPoseClient = new ROSLIB.Topic({
        ros : that.ros,
        name : '/camera/pose',
        messageType : 'geometry_msgs/Pose'
      });
      cameraPoseClient.subscribe(function(message) {
          if(that.on_camera_pose_received)
            that.on_camera_pose_received(message);
      });
      if(that.on_register_nodes)
          that.on_register_nodes();
      that.nodesRegistered = true;
    };
    
    this.waitForJsonProlog = function () {
        var client = new JsonProlog(that.ros, {});
        client.jsonQuery("true", function(result) {
            client.finishClient();
            if(result.error) {
                // Service does not exist
                setTimeout(that.waitForJsonProlog, 500);
            }
            else {
                that.hidePageOverlay();
                that.isPrologConnected = true;
            }
        });
    };
    
    ///////////////////////////////
    //////////// Marker Visualization
    ///////////////////////////////
    
    this.newProlog = function() {
        return that.ros ? new JsonProlog(that.ros, {}) : undefined;
    };
    
    this.newCanvas = function(options) {
        var x = new KnowrobCanvas(that, options);
        // connect to render event, dispatch to marker clients
		// FIXME TypeError: x.rosViewer.on is not a function
        //x.rosViewer.on('render', function(e) {
        //    if(that.markerClient)      that.markerClient.emit('render', e);
        //    if(that.markerArrayClient) that.markerArrayClient.emit('render', e);
        //});
        return x;
    };
    
    this.newDataVis = function(options) {
        return new DataVisClient(options);
    };
    
    this.newTaskTreeVis = function(options) {
        return new TaskTreeVisClient(options);
    };
    
    this.selectMarker = function(marker) {
        if(that.selectedMarker == marker)
          return;
        that.selectedMarker = marker;
        // inform the active iframe about selection (e.g., to show object query library)
        if(that.on_select_marker)
          that.on_select_marker(marker);
    };
    
    this.unselectMarker = function() {
      if(!that.selectedMarker)
        return;
      if(that.on_unselect_marker)
        that.on_unselect_marker(that.selectedMarker);
      that.selectedMarker = undefined;
    };
    
    this.removeMarker = function(marker) {
        if(marker === that.selectedMarker) {
            that.unselectMarker();
        }
        if(that.on_remove_marker)
            that.on_remove_marker(marker);
    };
    
    this.showMarkerMenu = function(marker) {
        if(that.on_show_marker_menu)
          that.on_show_marker_menu(marker);
    };
    
    this.on_render = function(camera,scene) {
        var index;
        for(index = 0; index < sprites.length; index++) {
            //sprites[index].camera = camera;
            //render_event.target = sprites[index];
            render_event.camera = camera;
            sprites[index].dispatchEvent(render_event);
        }
    };
    
    ///////////////////////////////
    //////////// Frame Overlay
    ///////////////////////////////
    
    this.createOverlay = function() {
        // Create page iosOverlay
        var page = document.getElementById('page');
        if(page) {
            var pageOverlay = document.createElement("div");
            pageOverlay.setAttribute("id", "page-overlay");
            pageOverlay.className = "ios-overlay ios-overlay-hide div-overlay";
            pageOverlay.innerHTML += '<span class="title"></span';
            pageOverlay.style.display = 'none';
            page.appendChild(pageOverlay);
            var spinner = createSpinner();
            pageOverlay.appendChild(spinner.el);
        }
    };
    
    this.showPageOverlay = function(text) {
      var pageOverlay = document.getElementById('page-overlay');
      if(pageOverlay && !that.pageOverlayDisabled) {
          pageOverlay.children[0].innerHTML = text;
          pageOverlay.style.display = 'block';
          pageOverlay.className = pageOverlay.className.replace("hide","show");
          pageOverlay.style.pointerEvents = "auto";
          that.pageOverlayDisabled = true;
      }
    };
    
    this.hidePageOverlay = function() {
      var pageOverlay = document.getElementById('page-overlay');
      if(pageOverlay && that.pageOverlayDisabled) {
          //pageOverlay.style.display = 'none';
          pageOverlay.className = pageOverlay.className.replace("show","hide");
          pageOverlay.style.pointerEvents = "none";
          that.pageOverlayDisabled = false;
      }
    };
};
