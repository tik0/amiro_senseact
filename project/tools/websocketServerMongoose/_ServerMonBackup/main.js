/*
 * Author: Markus Gaffke
 * File: main.js
 * Purpose: connecting to Server and running the Webside
 * Last Changed: 05.07.2014
 *
 */


//================================================================GETING URL===============================================
 var url = 'ws://' + location.host + '/ws';

var countercam=0, counterinf=0;
//================================================================CAMERA & Infrared===================================================

    var socket_Camera_Infrared = new WebSocket(url);
    
    try {
        socket_Camera_Infrared.onopen = function() {
            var div = document.getElementById("WebsocketCamera");
            div.textContent = "Websocket Verbindung offen";
        	}
	 } catch(exception) {
        alert('<p>Error' + exception);
    }
	document.getElementById("Ired_0").textContent = " Step: 1";

        socket_Camera_Infrared.onmessage = function(msg) {  //on receiving a message
		document.getElementById("Ired_0").textContent = " Step: 2";
		if(in_control && !controlpannel_free){
			showControlpannel();
		controlpannel_free=true;
		}
		document.getElementById("Ired_0").textContent = " Step: 3";
		//if in steering control, display more Information
		if(in_control){	document.getElementById("Speed_Status").textContent = "Geschwindigkeit: " + speedb.value + "%";
			document.getElementById("speeddiv").textContent = "Geschwindigkeit: "+ speedb.value + "%";
			document.getElementById("img_NO_CONTROL").style.zIndex='-10';			
			}
			if(speedb.value != speedbar_old && in_control){

			socket_Camera_Infrared.send(speedb.value);
			speedbar_old = speedb.value;			
			}
			if(speed_cam_send && speedIMGX >= 0 && in_control){
			socket_Camera_Infrared.send(200+speedIMGX);
			socket_Camera_Infrared.send(500+speedIMGY);				
			stop_signal_send = false;
			}

            var div = document.getElementById("WebsocketCamera");
            if (msg.data instanceof Blob) { //checking if the received data is a picture
                var canvas = document.getElementById('image');
                var context = canvas.getContext('2d');
                var image = new Image();
                image.onload = function() {
                    canvas.width = image.width;
                    canvas.height = image.height;
                    context.drawImage(image, 0, 0);
                };
                var reader = new FileReader();
                reader.onload = function(e) {
                    image.src = e.target.result;
                };
                reader.readAsDataURL(msg.data);
            } else{ //if data is not a picture, data will be processed here
		var infraredData = msg.data.split(":"); // getting the InfraredSensor Information out of the received data
			
			var whoIsInControlAndLight = msg.data.split(";"); // getting the User and Light Information out of the received data
			
			document.getElementById("User_Headcount").textContent = " Anzahl Nutzer:" +whoIsInControlAndLight[3];
			
			User_Count = parseInt(whoIsInControlAndLight[2]);
			total_Number_of_Clients = parseInt(whoIsInControlAndLight[3]);
			if(isChecked == false){
				//receiving User number
				User_Number = parseInt(whoIsInControlAndLight[2]);
				document.getElementById("User_Number").textContent = "Deine Usernummer: " + User_Number;
				if(!isNaN(User_Number)){isChecked = true;}
				
				if(parseInt(infraredData[0]) ==1 ){ //checking which robot the system is running on
				document.getElementById("Robot_ID").textContent = "Roboter: Bebot";
				document.getElementById("bebot_div").style.visibility = "visible";
				bebot = true;
				document.getElementById("amiro_div").style.visibility = "hidden";
				} else {
				document.getElementById("Robot_ID").textContent = "Roboter: Amiro";
				document.getElementById("amiro_div").style.visibility = "visible";
				document.getElementById("bebot_div").style.visibility = "hidden";
				}
			}
			
			var send_UserNumber = User_Number +1200;
			socket_Camera_Infrared.send(send_UserNumber);
		
			if(User_Number == whoIsInControlAndLight[1]){  //checking if User is in control
			document.getElementById("leave_get_Control").value="Kontrolle abgeben";
			in_control = true;
			showControlpannel();
			document.getElementById("Steering_Status").textContent = "Du darfst steuern!";			
			}else{
			document.getElementById("Steering_Status").textContent = "Der Nutzer mit der Nummer "+ whoIsInControlAndLight[1] 
			+ " darf steuern!";
			}
//=============================================Updating Light  Information==============================================================
			whiteLight=parseInt(whoIsInControlAndLight[4]);
			redLight=parseInt(whoIsInControlAndLight[5]);
			greenLight=parseInt(whoIsInControlAndLight[6]);
			blueLight=parseInt(whoIsInControlAndLight[7]);
			getLightMessage();

//=============================================Infrared color update for Bebot==========================================================
		if(bebot) {						
			updateBebot(infraredData);
		}
//=============================================Infrared color update for Amiro==========================================================
		if(!bebot){						
			updateAmiro(infraredData);
        		
			}
		}
	}	



        socket_Camera_Infrared.onclose = function() {
            document.getElementById("status").textContent = "Websocket Verbindung geschlossen";
        }
   

//=========================================================LIGHT BUTTONS on click===================================================
	 window.onload = function() {
   

	document.getElementById("whiteLight").onclick = function() {
		if(in_control){		
			if(whiteLight==0){
			allLightsOut();
			document.getElementById("whiteLight").value="Weißes Licht an";
			getLightMessage();
			whiteLight=1;
			socket_Camera_Infrared.send(1105);
			socket_Camera_Infrared.send(1101);
			}else{
			allLightsOut();
			getLightMessage();
			whiteLight=0;
			socket_Camera_Infrared.send(1105);
			}
		}
	};	


   	document.getElementById("redLight").onclick = function() {
		if(in_control){
			if(redLight==0){
			allLightsOut();
			document.getElementById("redLight").value="Rotes Licht an";
			getLightMessage();
			redLight=1;
			socket_Camera_Infrared.send(1105);
			socket_Camera_Infrared.send(1102);
			}else{
			allLightsOut();
			getLightMessage();
			redLight=0;
			socket_Camera_Infrared.send(1105);
			}
		}
	};	


   	document.getElementById("greenLight").onclick = function() {
		if(in_control){
			if(greenLight==0){
			allLightsOut();
			document.getElementById("greenLight").value="Gruenes Licht an";
			getLightMessage();
			greenLight=1;
			socket_Camera_Infrared.send(1105);
			socket_Camera_Infrared.send(1103);
			}else{
			allLightsOut();
			getLightMessage();
			greenLight=0;
			socket_Camera_Infrared.send(1105);
			}
		}
	};	


   	document.getElementById("blueLight").onclick = function() {
		if(in_control){		
			if(blueLight==0){
			allLightsOut();
			document.getElementById("blueLight").value="Blaues Licht an";
			getLightMessage();
			blueLight=1;
			socket_Camera_Infrared.send(1105);
			socket_Camera_Infrared.send(1104);
			}else{
			allLightsOut();
			getLightMessage();
			blueLight=0;
			socket_Camera_Infrared.send(1105);
			}
		}
	};
//======================================Controlbutton klicked=======================================================================
	document.getElementById("leave_get_Control").onclick = function() {
		// if true, give up control over robot		
		if(in_control && total_Number_of_Clients !== 1 ){
		in_control = false;
		User_Number = User_Count +1;
		document.getElementById("leave_get_Control").value="In Warteschlange";
		document.getElementById("User_Number").textContent = "Deine Usernummer: " + User_Number;
		document.getElementById("img_NO_CONTROL").style.visibility ="visible";
		document.getElementById("img_NO_CONTROL").style.zIndex ="10";
		document.getElementById("imgnormal").style.visibility='hidden';		
		document.getElementById("img_OL").style.visibility='hidden';
		document.getElementById("img_O").style.visibility='hidden';
		document.getElementById("img_OR").style.visibility='hidden';
		document.getElementById("img_ML").style.visibility='hidden';
		document.getElementById("img_MR").style.visibility='hidden';
		document.getElementById("img_UL").style.visibility='hidden';
		document.getElementById("img_U").style.visibility='hidden';
		document.getElementById("img_UR").style.visibility='hidden';
		socket_Camera_Infrared.send(1108);
		} 
	};
//======================================Helpbutton klicked==========================================================================
	document.getElementById("help").onclick = function() {
		document.getElementById("help_Example").style.visibility='visible';
		document.getElementById("help_div").style.visibility='visible';
		document.getElementById("img_Help").style.visibility='visible';
	};

};



