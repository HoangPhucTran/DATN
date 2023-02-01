// #include <Arduino.h>

// #define LED 13

// void setup() {
//   // put your setup code here, to run once:
//   Serial.begin(115200);
//   pinMode(LED, OUTPUT);
// }

// void loop() {
//   // put your main code here, to run repeatedly:
//   digitalWrite(LED, HIGH);
//   Serial.println("LED is on");
//   delay(1000);
//   digitalWrite(LED, LOW);
//   Serial.println("LED is off");
//   delay(1000);
// }

#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>

// Setup Wifi
const char* ssid = "Phuc";
const char* password = "123456789";

// HTML web page
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>

<html lang="en">

<head>

    <meta charset="UTF-8">
    <meta http-equiv="X-UA-Compatible" content="IE=edge">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>BẢNG ĐIỀU KHIỂN</title>

    <link rel="stylesheet" href="https://cdn.jsdelivr.net/npm/bootstrap@5.0.0-beta3/dist/css/bootstrap.min.css">
    <link rel="stylesheet" href="https://cdnjs.cloudflare.com/ajax/libs/font-awesome/5.15.3/css/all.min.css">
    <style>
        *{
            box-sizing: border-box;
        }

        body{
            font-family: 'Roboto', sans-serif;
        }

        /*Nav Bar*/

        header{
            background-color: #22242A;
            box-shadow: 0px -5px 20px 2px #333;
            height: 100px;
            top: 0;
            left: 0;
            right: 0;
            position: sticky;
            display: flex;
            align-items: center;
            justify-content: space-between;
        }

        footer{
            background-color: #22242A;
            box-shadow: 0px 1px 20px 2px #333;
            height: 70px;
            bottom: 0;
            left: 0;
            right: 0;
            position: fixed;
            display: flex;
            align-items: center;
            justify-content: space-between;
        }

        #fter{
            font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
            font-weight: 1000;
            font-size: 30px;
            color: white;
            position: absolute;
            top: 50%;
            left: 50%;
            transform: translate(-50%, -50%);
        }

        .nav_center{
            font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
            font-weight: 1000;
            color: white;
            position: absolute;
            top: 50%;
            left: 50%;
            transform: translate(-50%, -50%);
        }

        /*End Nav Bar*/

        /* Sidebar */

            .sidebar{
                position: relative;
                width: 100%;
            }

            .navigation{
                position: fixed;
                width: 320px;
                height: 100%;
                background-color: #2f323a;
                transition: 0.5s;
                overflow: hidden;
            }

            .navigation #control{
                background: #60656e;
            }

            ul{
                list-style-type: none;
                margin: 0;
                padding: 0;
                position: absolute;
                top: 0px;
                left: 0px;
                width: 100%;
            }

            li{
                position: relative;
                width: 100%;
                list-style: none;
                color: white;
            }

            li:hover{
                background-color: #40444e;
            }

            .sidebar a{
                color: #fff;
                display: block;
                width: 100%;
                line-height: 100px;
                text-decoration: none;
                padding-left: 40px;
                box-sizing: border-box;
                font-size: 20px;
                font-weight: 600;
                transition: 0.5s;
                transition-property: background;
                transition: transform 0.5s;
            }
            
            .sidebar a:hover{
                background: #60656e;
                transform: scale(1.1);
                transition: all 0.5s;
                color: #fff;
            }

            .sidebar i{
                margin-right: 40px;
            }

        /* End Sidebar */

        /* Content */

        .content{
            font: 15px sans-serif;
            font-weight: 600;
            font-size: 30px;
            margin-left: 320px;
            background-position: center;
            background-size: cover;
            height: 85vh;
            /* transition: 0.5s; */
            background-color: #60656e;
            position: relative;
        }

        .overview{
            position: absolute;
            margin-left: 110px;
        }

        .overview h1{
            font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
            margin-top: 30px;
            font-weight: 1000;
            color: #22242A;
        }

        .wrapper img{
            width: 350px;
            border-radius: 20px;
            box-shadow: 5px 10px 10px 0 rgb(0,0,0,0.5);
            margin-bottom: 50px;
            position: absolute;
            left: 50%;
            transform: translate(-50%);
        }

        .wrapper{
            float: left; 
            width: 40%;
            padding: 50px;
            margin-left: 100px;
            margin-top: 50px;
            height: 650px;
            border-radius: 50px;
            background-color: white;
            box-shadow: 5px 10px 10px 0 rgb(0,0,0,0.5);
            display: flex;
            Align-items: center;
            justify-content: center;
            Flex-direction: column;

        }



        p{
            position: absolute;
            left: 50%;
            transform: translate(-50%);
            font-size: 20px;
            font-weight: 550;
            height: 100px;
            width: 500px;
            margin-top: 400px;
        }

        .switch {
            position: relative;
            display: inline-block;
            width: 60px;
            height: 34px;
        }
        
        .switch input { 
        opacity: 0;
        width: 0;
        height: 0;
        }

        .slider {
        position: absolute;
        cursor: pointer;
        top: 0;
        left: 0;
        right: 0;
        bottom: 0;
        background-color: #ccc;
        -webkit-transition: .4s;
        transition: .4s;
        }

        .slider:before {
        position: absolute;
        content: "";
        height: 26px;
        width: 26px;
        left: 4px;
        bottom: 4px;
        background-color: white;
        -webkit-transition: .4s;
        transition: .4s;
        }

        input:checked + .slider {
        background-color: #2196F3;
        }

        input:focus + .slider {
        box-shadow: 0 0 1px #2196F3;
        }

        input:checked + .slider:before {
        -webkit-transform: translateX(26px);
        -ms-transform: translateX(26px);
        transform: translateX(26px);
        }

        /* Rounded sliders */
        .slider.round {
        border-radius: 34px;
        }

        .slider.round:before {
        border-radius: 50%;
        }

        .wrapper .auto{
            align-items: center;
            display: flex;
            margin-bottom: 50px;
        }

        .wrapper .auto .autoname, switch{
            display: inline;
            margin-right: 10px;
        }

        .time-input {
            display: flex;
            flex-direction: row;
            margin-left: 20px;
        }

        .setuptime{
            align-items: center;
            justify-content: center;
            margin-bottom: 50px;
            margin-top: 10px;
        }

        .button-apply{
            margin-left: 224px;
            margin-top: 10px;
        }

        .input-time {
            margin:  16px;
        }

        .arrow-container {
            display: flex;
            flex-direction: column;
            gap: 10px;
            margin-top: 10px;
        }
        .arrow-row-one {
            display: flex;
            justify-content: center;
            margin-top: 10px;
        }

        .wrapper .dkhuongdi{
            margin-top: 50px;
        }

        /* End Content */
    </style>
</head>

<body onload="startTime()">
    <!-- Nav Bar -->
    <header>
        <h1 class="nav_center">HỆ THỐNG ĐIỀU KHIỂN ROBOT KHỬ KHUẨN</h1>
    </header>
    <!-- End Nav Bar -->

    <!-- Sidebar -->
    <div class="sidebar">
        <div class="navigation">
            <ul>
                <li>
                    <a id="control" class="title"><i class="fas fa-cog"></i>Bảng Điều Khiển</a>
                </li>
            </ul>
        </div>
    </div>
    <!-- End Sidebar -->

    <!-- Main Content -->
    <div class="content">
        <div class="wrapper">
            <div class="auto">
                <div class="autoname">CHẾ ĐỘ AUTO:</div>
                <label class="switch" id="switch1">
                    <input type="checkbox" id="switchMode1">
                    <span class="slider round"></span>
                </label>
            </div>

            <div class="auto">
                <div class="autoname">ON/OFF VÒI PHUN:</div>
                <label class="switch" id="switch2">
                    <input type="checkbox" id="switchMode2" onchange="Change_State_Checkbox(this)">
                    <span class="slider round"></span>
                </label>
            </div>
            <div>THỜI GIAN SETUP:</div>
            <form class="setuptime" onsubmit="event.preventDefault(); handleSubmit()" name="setupTime">
                <div class="time-input">
                    <div class="input-time">
                        <label for="inputHour">GIỜ: </label>
                        <input type="number" max="24" min="0" name="inputHour" id="inputHour" placeholder="0"
                            value="0" />
                    </div>
                    <div class="input-time">
                        <label for="inputMinute">PHÚT: </label>
                        <input type="number" max="60" min="0" name="inputMinute" id="inputMinute" placeholder="0"
                            value="0" />
                    </div>
                    <div class="input-time">
                        <label for="inputSecond">GIÂY: </label>
                        <input type="number" max="60" min="0" name="inputSecond" id="inputSecond" placeholder="0"
                            value="0" />
                    </div>
                </div>
                <div class="button-apply">
                    <input type="submit" class="btn btn-primary" id="button-apply" value="APPLY">
                </div>
            </form>
            <div class="countdown">
                <label id="showcountdown"></label>
            </div>
        </div>

        <div class="wrapper">
            <div class="auto">
                <div class="autoname">CHẾ ĐỘ MANUAL: </div>
                <label class="switch" id="switch3">
                    <input type="checkbox" id="switchMode3" onchange="Change_State_Checkbox(this)">
                    <span class="slider round"></span>
                </label>
            </div>

            <div class="auto">
                <div class="autoname">ON/OFF VÒI PHUN: </div>
                <label class="switch" id="switch4">
                    <input type="checkbox" id="switchMode4" onchange="Change_State_Checkbox(this)">
                    <span class="slider round"></span>
                </label>
                </label>
            </div>
            <div class="dkhuongdi">ĐIỀU KHIỂN HƯỚNG ĐI:</div>
            <div class="arrow-container">
                <div class="arrow-row-one"><button class="btn btn-primary" id="button-up" onmousedown="Press_Navigation('ON_UP');" ontouchstart="Press_Navigation('ON_UP');" onmouseup="Press_Navigation('OFF_UP');" ontouchend="Press_Navigation('OFF_UP');"><i class="fas fa-arrow-up"></i></button>
                </div>
                <div class="arrow-row-two">
                    <button class="btn btn-primary" id="button-left" onmousedown="Press_Navigation('ON_LEFT');" ontouchstart="Press_Navigation('ON_LEFT');" onmouseup="Press_Navigation('OFF_LEFT');" ontouchend="Press_Navigation('OFF_LEFT');"><i class="fas fa-arrow-left"></i></button>
                    <button class="btn btn-primary" id="button-down" onmousedown="Press_Navigation('ON_DOWN');" ontouchstart="Press_Navigation('ON_DOWN');" onmouseup="Press_Navigation('OFF_DOWN');" ontouchend="Press_Navigation('OFF_DOWN');"><i class="fas fa-arrow-down"></i></button>
                    <button class="btn btn-primary" id="button-right" onmousedown="Press_Navigation('ON_RIGHT');" ontouchstart="Press_Navigation('ON_RIGHT');" onmouseup="Press_Navigation('OFF_RIGHT');" ontouchend="Press_Navigation('OFF_RIGHT');"><i class="fas fa-arrow-right"></i></button>
                </div>
            </div>
        </div>
    </div>
    <!-- End Content -->

    <footer>
        <div>
            <div id="fter"></div>
        </div>
    </footer>

    <script>
        var interval;
        var interval1;
        let shownTimeCountDown = document.querySelector('#showcountdown');
        let autoMode = document.querySelector('#switchMode1');
        let voiphun1 = document.querySelector('#switchMode2');
        let manualMode = document.querySelector('#switchMode3');
        let voiphun2 = document.querySelector('#switchMode4');
        let upBtn = document.querySelector('#button-up');
        let leftBtn = document.querySelector('#button-left');
        let downBtn = document.querySelector('#button-down');
        let rightBtn = document.querySelector('#button-right');
        if (!manualMode.checked) {
            upBtn.disabled = true;
            downBtn.disabled = true;
            leftBtn.disabled = true;
            rightBtn.disabled = true;
            voiphun2.disabled = true;
        }
        autoMode.addEventListener('change', () => {
            if (autoMode.checked) {
                console.log("Checkbox is checked");
                if(manualMode.checked)
                {
                    manualMode.checked = false;
                    var xhr = new XMLHttpRequest();
                    xhr.open("GET", "/switchMode3_OFF", true);
                    xhr.send();
                    

                }
                if(voiphun2.checked)
                {   
                    voiphun2.checked = false;
                    var xhr = new XMLHttpRequest();
                    xhr.open("GET", "/switchMode4_OFF", true);
                    xhr.send();
                }
            } else {
                console.log("Checkbox is unchecked");
                clearInterval(interval);
                shownTimeCountDown.textContent = '';
                inputHour.value = 0;
                inputMinute.value = 0;
                inputSecond.value = 0;
                var xhr = new XMLHttpRequest();
                xhr.open("GET", "/switchMode1_OFF", true);
                xhr.send();
                if(voiphun1.checked)
                {
                    voiphun1.checked = false;
                    var xhr = new XMLHttpRequest();
                    xhr.open("GET", "/switchMode2_OFF", true);
                    xhr.send();
                }
            }
        })
        manualMode.addEventListener('change', function () {
            if (manualMode.checked) {
                upBtn.disabled = false;
                downBtn.disabled = false;
                leftBtn.disabled = false;
                rightBtn.disabled = false;
                voiphun2.disabled = false;
                if(autoMode.checked)
                {
                    autoMode.checked = false;
                    var xhr = new XMLHttpRequest();
                    xhr.open("GET", "/switchMode1_OFF", true);
                    xhr.send();
                    
                }
                if(voiphun1.checked)
                {
                    voiphun1.checked = false;
                    var xhr = new XMLHttpRequest();
                    xhr.open("GET", "/switchMode1_OFF", true);
                    xhr.send();
                }
            } else {
                upBtn.disabled = true;
                downBtn.disabled = true;
                leftBtn.disabled = true;
                rightBtn.disabled = true;
                if(voiphun2.checked) {
                    voiphun2.checked = false;
                    var xhr = new XMLHttpRequest();
                    xhr.open("GET", "/switchMode4_OFF", true);
                    xhr.send();
                }
                voiphun2.disabled = true;
            }
        })
        
        function handleSubmit() {
            clearInterval(interval)
            let formData = document.forms['setupTime'];
            let hours = inputHour.value;
            let minutes = inputMinute.value;
            let seconds = inputSecond.value;
            let duration = parseInt(hours) * 3600 + parseInt(minutes) * 60 + parseInt(seconds);
            startTimer(duration, shownTimeCountDown)
            autoMode.checked = true;
            var xhr = new XMLHttpRequest();
            xhr.open("GET", "/switchMode1_ON", true);
            xhr.send();
            if(manualMode.checked)
            {
                manualMode.checked = false;
                var xhr = new XMLHttpRequest();
                xhr.open("GET", "/switchMode3_OFF", true);
                xhr.send();
            }
            if(voiphun2.checked)
            {
                voiphun2.checked = false;
                var xhr = new XMLHttpRequest();
                xhr.open("GET", "/switchMode4_OFF", true);
                xhr.send();
            }
            if(voiphun1.checked)
            {
                var xhr = new XMLHttpRequest();
                xhr.open("GET", "/switchMode2_ON", true);
                xhr.send();
            }
            if(!voiphun1.checked)
            {
                var xhr = new XMLHttpRequest();
                xhr.open("GET", "/switchMode2_OFF", true);
                xhr.send();
            }
        }
        function startTimer(duration, display) {
            var timer = duration, hours, minutes, seconds;
            interval = setInterval(
                function () {
                    hours = parseInt(timer / 3600)
                    minutes = parseInt(timer % 3600 / 60)
                    seconds = parseInt(timer % 60);
                    hours = hours < 10 ? "0" + hours : hours;
                    minutes = minutes < 10 ? "0" + minutes : minutes;
                    seconds = seconds < 10 ? "0" + seconds : seconds;
                    display.textContent = "ROBOT SẼ DỪNG SAU: " + hours + ":" + minutes + ":" + seconds;
                    // Auto Mode: OFF
                    if (--timer < 0) {
                        clearInterval(interval);
                        timer = duration;
                        setTimeout(() => display.textContent = '', 1000)
                        autoMode.checked = false;
                        voiphun1.checked = false;
                        inputHour.value = 0;
                        inputMinute.value = 0;
                        inputSecond.value = 0;
                        // Send HTTP_GET request turn off Auto mode & Motor
                        var xhr = new XMLHttpRequest();
                        xhr.open("GET", "/switchMode1_OFF", true);
                        xhr.send();
                        var xhr = new XMLHttpRequest();
                        xhr.open("GET", "/switchMode2_OFF", true);
                        xhr.send();
                    }
                }, 1000);
        }
        // Footer time
        function startTime() {
            const today = new Date();
            let h = today.getHours();
            let m = today.getMinutes();
            let s = today.getSeconds();
            m = checkTime(m);
            s = checkTime(s);
            document.getElementById('fter').innerHTML = h + ":" + m + ":" + s;
            setTimeout(startTime, 1000);
        }
        function checkTime(i) {
            if (i < 10) { i = "0" + i };  // Add zero in front of numbers < 10
            return i;
        }
        // End of footer time
		
		function Press_Navigation(x) {
			var xhr = new XMLHttpRequest();
			xhr.open("GET", "/" + x, true);
			xhr.send();
		}
    function Change_State_Checkbox(element) {
    var xhr = new XMLHttpRequest();
    if(element.checked) {
        xhr.open("GET", "/" + element.id + "_ON", true); 
    }
    else {
        xhr.open("GET", "/" + element.id + "_OFF", true);
    }
    xhr.send();
    }
    </script>
</body>
</html>)rawliteral";

void notFound(AsyncWebServerRequest *request) {
  request->send(404, "text/plain", "Not found");
}

AsyncWebServer server(80);

void setup() {
  Serial.begin(9600);
  Serial2.begin(9600);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  if (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("WiFi Failed!");
    return;
  }
  Serial.println();
  Serial.print("ESP IP Address: http://");
  Serial.println(WiFi.localIP());
//   Serial2.print("ESP IP Address: http://");
//   Serial2.println(WiFi.localIP());
  
  // Send web page to client
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", index_html);
  });

  // Receive an HTTP GET request - Up button
  server.on("/ON_UP", HTTP_GET, [] (AsyncWebServerRequest *request) {
    Serial.print("ON_UP\n");
    // Send UART signal
    Serial2.write("w ");
    Serial.print("Sending UART signal: up 1\n");
    
    request->send(200, "text/plain", "ok");
  });

  // Receive an HTTP GET request - Up button
  server.on("/OFF_UP", HTTP_GET, [] (AsyncWebServerRequest *request) {
    Serial.print("OFF_UP\n");
    // Send UART signal
    Serial2.write("p ");
    Serial.print("Sending UART signal: up 0\n");

    request->send(200, "text/plain", "ok");
  });

  // Receive an HTTP GET request - Down button
  server.on("/ON_DOWN", HTTP_GET, [] (AsyncWebServerRequest *request) {
    Serial.print("ON_DOWN\n");
    // Send UART signal
    Serial2.write("s ");
    Serial.print("Sending UART signal: down 1\n");

    request->send(200, "text/plain", "ok");
  });

  // Receive an HTTP GET request - Down button
  server.on("/OFF_DOWN", HTTP_GET, [] (AsyncWebServerRequest *request) {
    Serial.print("OFF_DOWN\n");
    // Send UART signal
    Serial2.write("p ");
    Serial.print("Sending UART signal: down 0\n");

    request->send(200, "text/plain", "ok");
  });

  // Receive an HTTP GET request - Left button
  server.on("/ON_LEFT", HTTP_GET, [] (AsyncWebServerRequest *request) {
    Serial.print("ON_LEFT\n");
    // Send UART signal
    Serial2.write("a ");
    Serial.print("Sending UART signal: left 1\n");

    request->send(200, "text/plain", "ok");
  });

  // Receive an HTTP GET request - Left button
  server.on("/OFF_LEFT", HTTP_GET, [] (AsyncWebServerRequest *request) {
    Serial.print("OFF_LEFT\n");
    // Send UART signal
    Serial2.write("p ");
    Serial.print("Sending UART signal: left 0\n");

    request->send(200, "text/plain", "ok");
  });

  // Receive an HTTP GET request - Right button
  server.on("/ON_RIGHT", HTTP_GET, [] (AsyncWebServerRequest *request) {
    Serial.print("ON_RIGHT\n");
    // Send UART signal
    Serial2.write("d ");
    Serial.print("Sending UART signal: right 1\n");

    request->send(200, "text/plain", "ok");
  });

  // Receive an HTTP GET request - Right button
  server.on("/OFF_RIGHT", HTTP_GET, [] (AsyncWebServerRequest *request) {
    Serial.print("OFF_RIGHT\n");
    // Send UART signal
    Serial2.write("p ");
    Serial.print("Sending UART signal: right 0\n");
    
    request->send(200, "text/plain", "ok");
  });

  // Receive an HTTP GET request - AUTO Checkbox
  server.on("/switchMode1_ON", HTTP_GET, [] (AsyncWebServerRequest *request) {
    Serial.print("AUTO MODE: ON\n");
    // Send UART signal
    Serial2.write("l ");
    Serial.print("Sending UART signal: auto 1\n");

    request->send(200, "text/plain", "ok");
  });

  // Receive an HTTP GET request - AUTO Checkbox
  server.on("/switchMode1_OFF", HTTP_GET, [] (AsyncWebServerRequest *request) {
    Serial.print("AUTO MODE: OFF\n");
    // Send UART signal
    Serial2.write("p ");
    Serial.print("Sending UART signal: auto 0\n");
    
    request->send(200, "text/plain", "ok");
  });

  // Receive an HTTP GET request - Motor Checkbox
  server.on("/switchMode2_ON", HTTP_GET, [] (AsyncWebServerRequest *request) {
    Serial.print("MOTOR1: ON\n");
    // Send UART signal
    Serial2.write("b1");
    Serial.print("Sending UART signal: motor1 1\n");

    request->send(200, "text/plain", "ok");
  });

  // Receive an HTTP GET request - Motor Checkbox
  server.on("/switchMode2_OFF", HTTP_GET, [] (AsyncWebServerRequest *request) {
    Serial.print("MOTOR1: OFF\n");
    // Send UART signal
    Serial2.write("b0");
    Serial.print("Sending UART signal: motor1 0\n");
    
    request->send(200, "text/plain", "ok");
  });

  // Receive an HTTP GET request - MANUAL Checkbox
  server.on("/switchMode3_ON", HTTP_GET, [] (AsyncWebServerRequest *request) {
    Serial.print("MANUAL MODE: ON\n");
    // Send UART signal
    // Serial2.write("manual 1");
    Serial.print("Sending UART signal: manual 1\n");

    request->send(200, "text/plain", "ok");
  });

  // Receive an HTTP GET request - MANUAL Checkbox
  server.on("/switchMode3_OFF", HTTP_GET, [] (AsyncWebServerRequest *request) {
    Serial.print("MANUAL MODE: OFF\n");
    // Send UART signal
    // Serial2.write("manual 0");
    Serial.print("Sending UART signal: manual 0\n");

    request->send(200, "text/plain", "ok");
  });

  // Receive an HTTP GET request - Motor Checkbox
  server.on("/switchMode4_ON", HTTP_GET, [] (AsyncWebServerRequest *request) {
    Serial.print("MOTOR: ON\n");
    // Send UART signal
    Serial2.write("b1");
    Serial.print("Sending UART signal: motor2 1\n");

    request->send(200, "text/plain", "ok");
  });

  // Receive an HTTP GET request - Motor Checkbox
  server.on("/switchMode4_OFF", HTTP_GET, [] (AsyncWebServerRequest *request) {
    Serial.print("MOTOR: OFF\n");
    // Send UART signal
    Serial2.write("b0");
    Serial.print("Sending UART signal: motor2 0\n");
    
    request->send(200, "text/plain", "ok");
  });
  
  server.onNotFound(notFound);
  server.begin();
}

void loop() {
 
}