
/// html functions
let opened_pannel = "";
function open_pannel(ele) {
  const updrp = document.getElementById(ele);
  if (updrp.style.visibility === 'hidden' || updrp.style.visibility === '') {
    updrp.style.visibility = 'visible';
    if (opened_pannel != "") document.getElementById(opened_pannel).style.visibility = 'hidden';
    opened_pannel = ele;
  }
  else {
    updrp.style.visibility = 'hidden';
  }
}

function set_value_range_slider(ele) {
  document.getElementById(ele + "_val").innerText = document.getElementById(ele).value;
}

// const x_slider = document.getElementById('x_slider');
const atitude_slider = document.getElementById('atitude_slider');
const emergency_stop_btn = document.getElementById('EM_stop_btn');
emergency_stop_btn.addEventListener('click', () =>{
  drone_X = 0;
  drone_Y = 0;
  atitude_slider.value = 0;
  sendtows();
  wsSendData('runns');
  set_value_range_slider('atitude_slider');
});
// x_slider.addEventListener('change', () => {
//     x_slider.value = 0; // Reset to the initial value (50 in this case)
//     set_value_range_slider('x_slider');
//     sendtows();
// });

// atitude_slider.addEventListener('change', () => {
//   atitude_slider.value = 0; // Reset to the initial value (50 in this case)
//   set_value_range_slider('atitude_slider');
//   sendtows();
// });

let kpidvalues = [[0, 0, 0], [0, 0, 0], [0, 0, 0]];

function changeKpid() {
  const kpid_cat = document.getElementById('kpid_cat').value;
  let col = kpid_cat === "kpid" ? 0 : kpid_cat === "krRP" ? 1 : kpid_cat === "kryw" ? 2 : 0;

  document.getElementById('kp').value = kpidvalues[col][0]
  document.getElementById('ki').value = kpidvalues[col][1]
  document.getElementById('kd').value = kpidvalues[col][2]
}

////////////////////joystick /////////////////////////////
let drone_X = 0, drone_Y = 0;
var canvas, ctx, joyBox;

window.addEventListener('load', () => {

  canvas = document.getElementById('canvas');
  ctx = canvas.getContext('2d');
  joyBox = document.getElementById('joyBox');
  resize();

  joyBox.addEventListener('mousedown', startDrawing);
  joyBox.addEventListener('mouseup', stopDrawing);
  joyBox.addEventListener('mousemove', Draw);

  joyBox.addEventListener('touchstart', startDrawing);
  joyBox.addEventListener('touchend', stopDrawing);
  joyBox.addEventListener('touchcancel', stopDrawing);
  joyBox.addEventListener('touchmove', Draw);
  window.addEventListener('resize', resize);

  document.getElementById("x_coordinate").innerText = 0;
  document.getElementById("y_coordinate").innerText = 0;
  // document.getElementById("speed").innerText = 0;
  // document.getElementById("angle").innerText = 0;
});




var width, height, radius, runRadius, x_orig, y_orig;
function resize() {
  console.log(window.innerHeight, window.innerWidth)
  if (window.innerWidth > window.innerHeight * 1.5) {
    width = window.innerHeight / 1.5;
  }
  else {
    width = window.innerWidth / 1.6;
  }
  // width = 500;
  radius = 50;
  runRadius = radius * 3;
  height = width / 1;
  // height = radius * 7;
  ctx.canvas.width = width;
  ctx.canvas.height = height;
  background();
  joystick(width / 2, height / 2);
}

function background() {
  x_orig = width / 2;
  y_orig = height / 2;

  ctx.beginPath();
  ctx.arc(x_orig, y_orig, radius + 20, 0, Math.PI * 2, true);
  ctx.fillStyle = '#ECE5E5';
  ctx.fill();
}

function joystick(width, height) {
  ctx.beginPath();
  ctx.arc(width, height, radius, 0, Math.PI * 2, true);
  ctx.fillStyle = '#F08080';
  ctx.fill();
  ctx.strokeStyle = '#F6ABAB';
  ctx.lineWidth = 8;
  ctx.stroke();
}

let coord = { x: 0, y: 0 };
let paint = false;

function getPosition(event) {
  var mouse_x = event.clientX || event.touches[0].clientX;
  var mouse_y = event.clientY || event.touches[0].clientY;
  coord.x = mouse_x - canvas.offsetLeft;
  coord.y = mouse_y - canvas.offsetTop;
}

function is_it_in_the_circle() {
  var current_radius = Math.sqrt(Math.pow(coord.x - x_orig, 2) + Math.pow(coord.y - y_orig, 2));
  if (runRadius >= current_radius) return true
  else return false
}


function startDrawing(event) {
  paint = true;
  getPosition(event);
  if (is_it_in_the_circle()) {
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    background();
    joystick(coord.x, coord.y);
    Draw(event);
  }
}


function stopDrawing() {
  paint = false;
  ctx.clearRect(0, 0, canvas.width, canvas.height);
  background();
  joystick(width / 2, height / 2);
  document.getElementById("x_coordinate").innerText = 0;
  document.getElementById("y_coordinate").innerText = 0;
  // document.getElementById("speed").innerText = 0;
  // document.getElementById("angle").innerText = 0;
  drone_Y = 0;
  drone_X = 0;
    sendtows();
    // console.log(0,0);

}
function mapRange(value, inMin, inMax, outMin, outMax) {
  return (value - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}
function Draw(event) {

  if (paint) {
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    background();
    var angle_in_degrees, x, y, speed;
    var angle = Math.atan2((coord.y - y_orig), (coord.x - x_orig));

    // if (Math.sign(angle) == -1) {
    //     angle_in_degrees = Math.round(-angle * 180 / Math.PI);
    // }
    // else {
    //     angle_in_degrees =Math.round( 360 - angle * 180 / Math.PI);
    // }


    if (is_it_in_the_circle()) {
      joystick(coord.x, coord.y);
      x = coord.x;
      y = coord.y;
    }
    else {
      x = runRadius * Math.cos(angle) + x_orig;
      y = runRadius * Math.sin(angle) + y_orig;
      joystick(x, y);
    }


    getPosition(event);

    // var speed =  Math.round(100 * Math.sqrt(Math.pow(x - x_orig, 2) + Math.pow(y - y_orig, 2)) / radius);

    var x_relative = Math.round(x - x_orig);
    var y_relative = -Math.round(y - y_orig);

    x_relative = Math.round(mapRange(x_relative, -150, 150, -70, 70));
    y_relative = Math.round(mapRange(y_relative, -150, 150, -70, 70));

    drone_Y = y_relative;
    drone_X = x_relative;

    document.getElementById("x_coordinate").innerText = x_relative;
    document.getElementById("y_coordinate").innerText = y_relative;
    // document.getElementById("speed").innerText = speed;
    // document.getElementById("angle").innerText = angle_in_degrees;

    // send( x_relative,y_relative,speed,angle_in_degrees);

    sendtows();
    // console.log(x_relative, y_relative);
  }
}
///////////////////////

// const ws = new WebSocket('ws://192.168.238.130:81'); // Replace with your ESP32's WebSocket URL
// const ws = new WebSocket('ws://192.168.1.8:81'); // Replace with your ESP32's WebSocket URL
const ws = new WebSocket('ws://drone.local:81'); // Replace with your ESP32's WebSocket URL
// const ws = new WebSocket('ws://esp32-update.local:81'); // Replace with your ESP32's WebSocket URL
// const ws = new WebSocket('ws://192.168.1.9/ws'); // Replace with your ESP32's WebSocket URL
let binchuncksend = false;
let deb_consoleData = "";
function resetcon() {
  deb_consoleData = "";
  document.getElementById('deb_console').innerText = deb_consoleData;
}
ws.onopen = () => {
  console.log('Connected to WebSocket');
};

ws.onmessage = (event) => {
  if (event.data instanceof Blob) {
    event.data.arrayBuffer().then((wsBufArr) => {
      // const wsBufArr = new Uint8Array(buffer);
      let len = wsBufArr.length;
      if (len == 0) return;

      const decoderTXT = new TextDecoder('utf-8');
      let dtType = decoderTXT.decode(wsBufArr.slice(0, 4));
      console.log("dtype :" + dtType + "buf", wsBufArr.slice(4));

      if (dtType === "motc") {
        document.getElementById('calib_mot_1').value = binaryToInt(wsBufArr.slice(4, 6), 'i16');
        document.getElementById('calib_mot_2').value = binaryToInt(wsBufArr.slice(6, 8), 'i16');
        document.getElementById('calib_mot_3').value = binaryToInt(wsBufArr.slice(8, 10), 'i16');
        document.getElementById('calib_mot_4').value = binaryToInt(wsBufArr.slice(10, 12), 'i16');

      } else if (dtType === "motm") {
        document.getElementById('max_mot').value = binaryToInt(wsBufArr.slice(4, 8), 'i16');
      } else if (dtType === "kpid" || dtType === "krRP" || dtType === "kryw") {
        let col = dtType === "kpid" ? 0 : dtType === "krRP" ? 1 : dtType === "kryw" ? 2 : 0;

        kpidvalues[col][0] = binaryToInt(wsBufArr.slice(4, 8), 'f32');
        kpidvalues[col][1] = binaryToInt(wsBufArr.slice(8, 12), 'f32');
        kpidvalues[col][2] = binaryToInt(wsBufArr.slice(12, 16), 'f32');

        changeKpid();
        // document.getElementById('kp').value = binaryToInt(wsBufArr.slice(4,8), 'f32');
        // document.getElementById('ki').value = binaryToInt(wsBufArr.slice(8,12), 'f32');
        // document.getElementById('kd').value = binaryToInt(wsBufArr.slice(12,16), 'f32');
      }
      if (dtType === "mots") {
        document.getElementById('calib_mot_1').value = binaryToInt(wsBufArr.slice(4, 6), 'i16');
        document.getElementById('calib_mot_2').value = binaryToInt(wsBufArr.slice(6, 8), 'i16');
        // document.getElementById('calib_mot_3').value = binaryToInt(wsBufArr.slice(8,12), 'i16');
        // document.getElementById('calib_mot_4').value = binaryToInt(wsBufArr.slice(12,16), 'i16');
      }


    }).catch((error) => console.log("error blob : ", error));
  } else {
    console.log('rx msg: ', event.data);

    let ws_dt_type = event.data.slice(0, 3);
    let ws_data = event.data.slice(3);

    if (ws_dt_type == "OTA") {
      if (ws_data == "wschunk") binchuncksend = false;
      else if (ws_data[0] == "P") document.getElementById('progress').innerText = 'ESP update ' + ws_data.slice(1) + ' %';
    }
    else if (ws_dt_type == "CMD") document.getElementById('progress').innerText = ws_data;
    else if (ws_dt_type == "JUN") document.getElementById('junjun').innerText = ws_data;
    else if (ws_dt_type == "JUC") document.getElementById('jucjuc').innerText = ws_data;
    else if (ws_dt_type == "DEB") {
      if (deb_consoleData.length > 10000) deb_consoleData = deb_consoleData.slice(1000);
      deb_consoleData += ws_data;
      document.getElementById('deb_console').innerText = deb_consoleData;
    }
  }
};

ws.onclose = () => {
  console.log('WebSocket connection closed');
};
function binaryToInt(bin, type) {
  const view = new DataView(bin);
  if (type === 'ui8') {
    return view.getUint8(0);
  } else if (type === 'ui16') {
    return view.getUint16(0, true); // Big-endian
  } else if (type === 'i16') {
    return view.getInt16(0, true); // Big-endian
  } else if (type === 'ui32') {
    return view.getUint32(0, true); // Big-endian
  } else if (type === 'i32') {
    return view.getInt32(0, true); // Big-endian
  } else if (type === 'f32') {
    return view.getFloat32(0, true);
  }
  return 0;
}
function intToByteArrayTyped(value, byteCount, type) {
  const buffer = new ArrayBuffer(byteCount);
  const view = new DataView(buffer);

  // Store the integer in the buffer as needed
  if (type === 'ui8') {
    view.setUint8(0, value);
  } else if (type === 'ui16') {
    view.setUint16(0, value, true); // Big-endian
  } else if (type === 'i16') {
    view.setInt16(0, value, true); // Big-endian
  } else if (type === 'ui32') {
    view.setUint32(0, value, true); // Big-endian
  } else if (type === 'i32') {
    view.setInt32(0, value, true); // Big-endian
  } else if (type === 'f32') {
    view.setFloat32(0, value, true);
  }

  // Convert the buffer to a Uint8Array (byte array)
  return new Uint8Array(buffer);
  // return Array.from(new Uint8Array(buffer));
  // const tempbytearr = Array.from(new Uint8Array(buffer));
  // return tempbytearr.map(byte => String.fromCharCode(byte)).join('');
}
const encoderTXT = new TextEncoder();
function getWsSendBuffer(HEAD, CMD, ...DATA) {
  const wsHeader = HEAD + CMD;
  const wsHeaderbyt = encoderTXT.encode(wsHeader);

  let totalbyteLen = DATA.reduce((acc, curr) => acc + curr.length, 0);
  totalbyteLen += wsHeaderbyt.length;

  let databytes = new Uint8Array(totalbyteLen);
  databytes.set(wsHeaderbyt, 0);

  let byteOffset = wsHeaderbyt.length;
  for (let data of DATA) {
    databytes.set(data, byteOffset);
    byteOffset += data.length;
  }

  return databytes;
}

document.getElementById('uploadButton').addEventListener('click', () => {
  const fileInput = document.getElementById('fileInput');
  if (fileInput.files.length === 0) {
    alert('Please select a file');
    return;
  }

  const file = fileInput.files[0];
  const reader = new FileReader();

  reader.onload = (e) => {
    const data = new Uint8Array(e.target.result);
    const wsotahead = encoderTXT.encode("ota1");
    let offset = 0;
    const chunkSize = 1024 * 16;
    let start_ota = false;

    const sendNextChunk = () => {
      if (offset == 0 && !start_ota) {
        const bytesss = getWsSendBuffer("ota0", "", intToByteArrayTyped(data.length, 4, 'ui32'));
        console.log(bytesss);
        ws.send(bytesss);

        start_ota = true;
        binchuncksend = true;
      }
      if (!binchuncksend) {
        if (offset < data.length) {
          const datachunk = data.slice(offset, offset + chunkSize);
          let chunk = new Uint8Array(datachunk.length + 4);
          chunk.set(wsotahead, 0);
          chunk.set(datachunk, 4);

          ws.send(chunk);
          offset += chunkSize;

          const progress = ((offset / data.length) * 100).toFixed(2);

          document.getElementById('progressupload').innerText = `Progress: ${progress}% | ${offset / 1024}/${data.length / 1024} KB`;

          binchuncksend = true;
        } else {
          console.log('Upload complete');
          document.getElementById('progressupload').innerText = 'Upload complete';
        }
      }
      if (offset < data.length) setTimeout(sendNextChunk, 10); // Delay to avoid flooding the WebSocket
    };
    sendNextChunk();
  };

  reader.readAsArrayBuffer(file);
});
document.getElementById('espReset').addEventListener('click', () => {
  ws.send("rest");
  console.log("rest");
});


function wsSendData(elm) {

  if (elm == 'runnn') {
    console.log(getWsSendBuffer("data", "runn", intToByteArrayTyped(document.getElementById('runnum').value, 1, 'ui8')));
    ws.send(getWsSendBuffer("data", "runn", intToByteArrayTyped(document.getElementById('runnum').value, 1, 'ui8')));
  } else if (elm == 'runnr') {
    console.log(getWsSendBuffer("data", "runn", intToByteArrayTyped(document.getElementById('rrun').value, 1, 'ui8')));
    ws.send(getWsSendBuffer("data", "runn", intToByteArrayTyped(document.getElementById('rrun').value, 1, 'ui8')));
  } else if (elm == 'runns') {
    ws.send(getWsSendBuffer("data", "runn", intToByteArrayTyped(document.getElementById('rStop').value, 1, 'ui8')));
  } else if (elm == 'runnc') {
    // console.log(getWsSendBuffer("data", "runn", intToByteArrayTyped(roboRun, 1, 'ui8')));
    ws.send(getWsSendBuffer("data", "runn", intToByteArrayTyped(document.getElementById('calib').value, 1, 'ui8')));
  } else if (elm == 'swit') {
    ws.send("swit");
  }

  else if (elm == 'kpid') {
    const kpid_cat = document.getElementById('kpid_cat').value;
    console.log(kpid_cat);

    let col = kpid_cat === "kpid" ? 0 : kpid_cat === "krRP" ? 1 : kpid_cat === "kryw" ? 2 : 0;

    kpidvalues[col][0] = document.getElementById('kp').value;
    kpidvalues[col][1] = document.getElementById('ki').value;
    kpidvalues[col][2] = document.getElementById('kd').value;

    const buf = getWsSendBuffer("data", kpid_cat,
      intToByteArrayTyped(document.getElementById('kp').value, 4, 'f32'),
      intToByteArrayTyped(document.getElementById('ki').value, 4, 'f32'),
      intToByteArrayTyped(document.getElementById('kd').value, 4, 'f32'));

    console.log(buf);
    ws.send(buf);

  } else if (elm == 'motc') {
    const buf = getWsSendBuffer("data", "motc",
      intToByteArrayTyped(document.getElementById('calib_mot_1').value, 2, 'i16'),
      intToByteArrayTyped(document.getElementById('calib_mot_2').value, 2, 'i16'),
      intToByteArrayTyped(document.getElementById('calib_mot_3').value, 2, 'i16'),
      intToByteArrayTyped(document.getElementById('calib_mot_4').value, 2, 'i16'));
    console.log(buf);
    ws.send(buf);
  } else if (elm == 'motm') {
    const buf = getWsSendBuffer("data", "motm",
      intToByteArrayTyped(document.getElementById('max_mot').value, 2, 'i16'),
      intToByteArrayTyped(document.getElementById('max_mot').value, 2, 'i16'),
      intToByteArrayTyped(document.getElementById('max_mot').value, 2, 'i16'),
      intToByteArrayTyped(document.getElementById('max_mot').value, 2, 'i16'));
    console.log(buf);
    ws.send(buf);
  }
}
function sendtows() {
  let x_slider_val = drone_X;
  let atitude_slider_val = atitude_slider.value;
  let y_slider_val = drone_Y;
  // console.log(x_slider_val + "  " + atitude_slider_val);
  const buf = getWsSendBuffer("driv", "",
    intToByteArrayTyped(x_slider_val, 2, 'i16'),
    intToByteArrayTyped(atitude_slider_val, 2, 'i16'),
    intToByteArrayTyped(y_slider_val, 2, 'i16'));
  // console.log(buf);
  ws.send(buf);
}